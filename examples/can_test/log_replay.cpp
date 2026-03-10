/*=+--+=#=+--         SwiftCore Flight Management Software        --+=#=+--+=#*\
|               Copyright (C) 2015 Black Swift Technologies LLC.               |
|                             All Rights Reserved.                             |

     NOTICE:  All information contained herein is, and remains the property
     of Black Swift Technologies.

     The intellectual and technical concepts contained herein are
     proprietary to Black Swift Technologies LLC and may be covered by U.S.
     and foreign patents, patents in process, and are protected by trade
     secret or copyright law.

     Dissemination of this information or reproduction of this material is
     strictly forbidden unless prior written permission is obtained from
     Black Swift Technologies LLC.
|                                                                              |
|                                                                              |
\*=+--+=#=+--                 --+=#=+--+=#=+--                    --+=#=+--+=#*/
#include "log_replay.h"
#include "main.h"
#include "test.h"
#include "bridge.h"
#include "comm_packets.h"
#include "bst_packet.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <termios.h>
#include <sys/select.h>

using namespace bst::comms;

// ── user-specified time bounds (seconds relative to log start, -1 = unset) ──
float replay_start_s = -1.0f;
float replay_stop_s  = -1.0f;

// ── log entry ────────────────────────────────────────────────────────────────
typedef struct {
	float    time_s;       // seconds from State_t.system_time
	uint16_t usec[16];     // actuator microseconds (cast from int16_t)
} LogActuatorEntry_t;

// ── forward declarations ─────────────────────────────────────────────────────
static bool     parseLogFile(const char * filename,
                             LogActuatorEntry_t ** entries, uint32_t * count);
static void     sendSafetyOff(void);
static void     printProgress(float cur_s, float total_s,
                              const uint16_t * usec, bool paused);
static bool     replayInputAvailable(void);

// ── progress bar width ───────────────────────────────────────────────────────
#define BAR_WIDTH 40

// ── key codes ────────────────────────────────────────────────────────────────
#define KEY_ESC    27
#define KEY_CTRL_C 3
#define KEY_SPACE  ' '

// ── skip amount ──────────────────────────────────────────────────────────────
#define SKIP_SEC 10.0f

// ─────────────────────────────────────────────────────────────────────────────
//  parseLogFile  –  read a BST .bin log and extract actuator entries
//
//  Binary log packets (no addressing):
//    [0x55][0x24][TYPE][ACTION][SIZE_L][SIZE_H][DATA…][CHK_L][CHK_H]
//
//  We track the most recent State_t.system_time (type 16, first float)
//  and pair it with each ACTUATORS_VALUES packet (type 48).
// ─────────────────────────────────────────────────────────────────────────────
static bool parseLogFile(const char * filename,
                         LogActuatorEntry_t ** entries, uint32_t * count)
{
	FILE * fp = fopen(filename, "rb");
	if(!fp) {
		printf("ERROR: cannot open %s\n", filename);
		return false;
	}

	// get file size
	fseek(fp, 0, SEEK_END);
	long file_size = ftell(fp);
	fseek(fp, 0, SEEK_SET);

	uint8_t * buf = (uint8_t *)malloc(file_size);
	if(!buf) {
		printf("ERROR: cannot allocate %ld bytes\n", file_size);
		fclose(fp);
		return false;
	}

	long read_sz = fread(buf, 1, file_size, fp);
	fclose(fp);

	if(read_sz != file_size) {
		printf("ERROR: short read (%ld / %ld)\n", read_sz, file_size);
		free(buf);
		return false;
	}

	// ── first pass: count actuator packets ────────────────────────────────
	uint32_t n_act = 0;
	long i = 0;
	while(i < file_size - 8) {
		if(buf[i] == 0x55 && buf[i+1] == 0x24) {
			uint8_t  pkt_type = buf[i+2];
			uint16_t pkt_size = buf[i+4] | ((uint16_t)buf[i+5] << 8);
			long     total    = 6 + pkt_size + 2;

			if(i + total > file_size) break;

			if(pkt_type == ACTUATORS_VALUES && pkt_size == 32)
				n_act++;

			i += total;
		} else {
			i++;
		}
	}

	if(n_act == 0) {
		printf("ERROR: no ACTUATORS_VALUES packets found in %s\n", filename);
		free(buf);
		return false;
	}

	// ── allocate ──────────────────────────────────────────────────────────
	LogActuatorEntry_t * e = (LogActuatorEntry_t *)malloc(
			n_act * sizeof(LogActuatorEntry_t));
	if(!e) {
		printf("ERROR: cannot allocate entries\n");
		free(buf);
		return false;
	}

	// ── second pass: extract ──────────────────────────────────────────────
	float    last_state_time = 0.0f;
	uint32_t idx = 0;
	i = 0;
	while(i < file_size - 8 && idx < n_act) {
		if(buf[i] == 0x55 && buf[i+1] == 0x24) {
			uint8_t  pkt_type = buf[i+2];
			uint16_t pkt_size = buf[i+4] | ((uint16_t)buf[i+5] << 8);
			long     total    = 6 + pkt_size + 2;

			if(i + total > file_size) break;

			// State_t – first field is float system_time
			if(pkt_type == STATE_STATE && pkt_size >= 4) {
				memcpy(&last_state_time, &buf[i+6], sizeof(float));
			}

			// Actuators_t – int16_t usec[16]
			if(pkt_type == ACTUATORS_VALUES && pkt_size == 32) {
				e[idx].time_s = last_state_time;

				int16_t raw[16];
				memcpy(raw, &buf[i+6], 32);
				for(uint8_t ch = 0; ch < 16; ch++)
					e[idx].usec[ch] = (uint16_t)raw[ch];

				idx++;
			}

			i += total;
		} else {
			i++;
		}
	}

	free(buf);

	*entries = e;
	*count   = idx;
	return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  sendSafetyOff  –  zero actuators (throttle=1000, servos=1500)
// ─────────────────────────────────────────────────────────────────────────────
static void sendSafetyOff(void)
{
	zeroAcutators();
}

// ─────────────────────────────────────────────────────────────────────────────
//  printProgress  –  single-line progress bar + 16 channel values
// ─────────────────────────────────────────────────────────────────────────────
static void printProgress(float cur_s, float total_s,
                          const uint16_t * usec, bool paused)
{
	float frac = (total_s > 0.0f) ? cur_s / total_s : 0.0f;
	if(frac < 0.0f) frac = 0.0f;
	if(frac > 1.0f) frac = 1.0f;

	int filled = (int)(frac * BAR_WIDTH);

	// time strings
	int cur_min  = (int)cur_s / 60;
	int cur_sec  = (int)cur_s % 60;
	int tot_min  = (int)total_s / 60;
	int tot_sec  = (int)total_s % 60;

	// line 1: progress bar
	printf("\r\033[K");  // clear line
	printf("%s [", paused ? "PAUSED" : "PLAY  ");
	for(int j = 0; j < BAR_WIDTH; j++)
		printf("%c", j < filled ? '#' : '-');
	printf("] %d:%02d / %d:%02d  ", cur_min, cur_sec, tot_min, tot_sec);

	// line 2: actuator values
	printf("\n\033[K");
	printf("  CH: ");
	for(uint8_t ch = 0; ch < 16; ch++)
		printf("%04u ", usec[ch]);

	// move cursor back up one line
	printf("\033[1A\r");
	fflush(stdout);
}

// ─────────────────────────────────────────────────────────────────────────────
static bool replayInputAvailable(void)
{
	struct timeval tv;
	fd_set fds;
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds);
	select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
	return FD_ISSET(0, &fds);
}

// ─────────────────────────────────────────────────────────────────────────────
//  runLogReplay  –  main replay loop
//
//  Returns true  → go back to interactive menu
//  Returns false → full quit (sets running=false before returning)
// ─────────────────────────────────────────────────────────────────────────────
bool runLogReplay(const char * filename)
{
	LogActuatorEntry_t * entries = NULL;
	uint32_t             n_entries = 0;

	printf("\nParsing log file: %s ...\n", filename);

	if(!parseLogFile(filename, &entries, &n_entries))
		return true;   // parsing failed – back to menu

	float log_first = entries[0].time_s;
	float log_last  = entries[n_entries - 1].time_s;

	printf("Loaded %u actuator entries, %.1f s total (%.1f - %.1f s)\n",
			n_entries, log_last - log_first, log_first, log_last);

	// ── apply --ss / --tt / --dd trim ────────────────────────────────────
	float win_start = log_first;
	float win_end   = log_last;

	if(replay_start_s >= 0.0f)
		win_start = log_first + replay_start_s;
	if(replay_stop_s >= 0.0f)
		win_end = log_first + replay_stop_s;

	// clamp to actual data range
	if(win_start < log_first) win_start = log_first;
	if(win_end   > log_last)  win_end   = log_last;
	if(win_start > log_last)  win_start = log_last;
	if(win_end   < win_start) win_end   = win_start;

	// find first and last entry indices within the window
	uint32_t first_idx = 0;
	while(first_idx < n_entries && entries[first_idx].time_s < win_start)
		first_idx++;
	if(first_idx > 0 && entries[first_idx].time_s > win_start)
		first_idx--;

	uint32_t last_idx = n_entries - 1;
	while(last_idx > first_idx && entries[last_idx].time_s > win_end)
		last_idx--;

	// shift view to trimmed window
	LogActuatorEntry_t * win_entries = entries + first_idx;
	uint32_t             win_count   = last_idx - first_idx + 1;

	float start_time = win_entries[0].time_s;
	float end_time   = win_entries[win_count - 1].time_s;
	float duration   = end_time - start_time;

	int ss_min = (int)(start_time - log_first) / 60;
	int ss_sec = (int)(start_time - log_first) % 60;
	int ee_min = (int)(end_time - log_first) / 60;
	int ee_sec = (int)(end_time - log_first) % 60;

	printf("Replay window: %d:%02d - %d:%02d  (%u entries, %.1f s)\n",
			ss_min, ss_sec, ee_min, ee_sec, win_count, duration);

	printf("\nReplay controls:\n");
	printf("  SPACE  : pause / resume\n");
	printf("  >      : skip forward  %ds\n", (int)SKIP_SEC);
	printf("  <      : skip backward %ds\n", (int)SKIP_SEC);
	printf("  ESC    : stop replay, return to menu\n");
	printf("  q/^C   : quit program (motors off)\n");
	printf("\nStarting replay ...\n\n");

	// clock reference
	struct timespec ts_start;
	clock_gettime(CLOCK_MONOTONIC, &ts_start);

	bool paused       = false;
	float pause_offset = 0.0f;  // accumulated pause time
	struct timespec ts_pause_start;

	float log_offset = 0.0f;    // seek offset from user < / >

	uint32_t idx = 0;
	bool     quit_program = false;
	bool     stop_replay  = false;

	while(!stop_replay && !quit_program && idx < win_count) {
		// ── handle keyboard ──────────────────────────────────────────
		if(replayInputAvailable()) {
			char ch = getchar();

			if(ch == KEY_ESC) {
				stop_replay = true;
			} else if(ch == KEY_CTRL_C || ch == 'q') {
				quit_program = true;
			} else if(ch == KEY_SPACE) {
				if(paused) {
					struct timespec ts_now;
					clock_gettime(CLOCK_MONOTONIC, &ts_now);
					pause_offset += (ts_now.tv_sec - ts_pause_start.tv_sec)
						+ (ts_now.tv_nsec - ts_pause_start.tv_nsec) / 1e9f;
					paused = false;
				} else {
					clock_gettime(CLOCK_MONOTONIC, &ts_pause_start);
					paused = true;
				}
			} else if(ch == '>' || ch == '.') {
				log_offset += SKIP_SEC;
			} else if(ch == '<' || ch == ',') {
				log_offset -= SKIP_SEC;
			}
		}

		if(paused) {
			if(idx < win_count)
				BRIDGE_SendActuatorPkt(1, win_entries[idx].usec);

			printProgress(win_entries[idx].time_s - start_time, duration,
					win_entries[idx].usec, true);
			usleep(20000);
			continue;
		}

		// ── compute current log time ─────────────────────────────────
		struct timespec ts_now;
		clock_gettime(CLOCK_MONOTONIC, &ts_now);

		float wall_elapsed = (ts_now.tv_sec - ts_start.tv_sec)
			+ (ts_now.tv_nsec - ts_start.tv_nsec) / 1e9f;
		float play_time = wall_elapsed - pause_offset + log_offset;

		float target_log_time = start_time + play_time;

		// clamp to window
		if(target_log_time < start_time) {
			log_offset += (start_time - target_log_time);
			target_log_time = start_time;
		}
		if(target_log_time > end_time) {
			// past the end of the window – done
			break;
		}

		// ── advance index to match target time ───────────────────────
		while(idx < win_count - 1 && win_entries[idx + 1].time_s <= target_log_time)
			idx++;
		while(idx > 0 && win_entries[idx].time_s > target_log_time)
			idx--;

		// ── send actuator command ────────────────────────────────────
		BRIDGE_SendActuatorPkt(1, win_entries[idx].usec);

		// ── display ──────────────────────────────────────────────────
		printProgress(win_entries[idx].time_s - start_time, duration,
				win_entries[idx].usec, false);

		// ── sleep to maintain ~100 Hz loop rate ──────────────────────
		usleep(10000);
	}

	// ── cleanup: always send safety-off on exit ──────────────────────────
	printf("\n\n");
	sendSafetyOff();

	if(quit_program) {
		printf("Replay stopped – motors off, quitting.\n");
		running = false;
		free(entries);
		return false;
	}

	if(stop_replay) {
		printf("Replay stopped – motors off, returning to menu.\n");
	} else {
		printf("Replay complete – motors off.\n");
	}

	free(entries);
	return true;
}
