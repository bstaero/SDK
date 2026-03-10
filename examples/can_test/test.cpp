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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "test.h"
#include "main.h"
#include "structs.h"

#include "debug.h"
#include "flight_plan.h"

#include "bridge.h"
#include "log_replay.h"


// variables
volatile bool display_telemetry = false;
volatile bool send_actuators = false;
volatile bool write_file = false;

bool show_gps = false;
bool show_mag = false;
bool show_dynamic = false;
bool show_static = false;

bool new_gps = false;
bool new_mag = false;
bool new_dynamic = false;
bool new_static = false;

bool print_timing = false;

bool auto_test = false;

extern uint32_t gnss_lla_cnt;
extern uint32_t gnss_utc_cnt;
extern uint32_t gnss_vel_cnt;
extern uint32_t gnss_hs_cnt;

extern uint32_t mag_cnt;

extern uint32_t stat_p_cnt;

#define TRIGGER_LENGTH 1.0

#define CMD_BUF_SIZE 8
Packet cmd_buf[CMD_BUF_SIZE];
uint8_t cmd_buf_start = 0;
uint8_t cmd_buf_end = 0;

#define PKT_BUF_SIZE 8
Packet pkt_buf[PKT_BUF_SIZE];
uint8_t pkt_buf_start = 0;
uint8_t pkt_buf_end = 0;

static uint8_t actuator_types[16];
static uint16_t actuators[16];

extern CommunicationsInterface * comm_interface;

// functional definitions

// packet for transmision
Packet              tx_packet;

extern char log_filename[];

void printTestHelp() {
	printf("Keys:\n");
	printf("  t        : Toggle telemetry display\n");
	printf("\n");
	printf("  T        : take a picture\n");
	printf("  0 to A   : test channel n\n");
	printf("  f        : replay actuators from log file\n");
	printf("\n");
	printf("  p        : print this help\n");
}

void zeroAcutators() {
	for(uint8_t i=0; i<16; i++) {
		if( (actuator_types[i] == ACT_L_THROTTLE) ||
				(actuator_types[i] == ACT_R_THROTTLE) ||
				(actuator_types[i] == ACT_ROTOR) ) {
			actuators[i] = 1000;
		} else {
			actuators[i] = 1500;
		}
	}
	BRIDGE_SendActuatorPkt(1, actuators);
}


void updateTest() {
	char input; 
	static char auto_char = '0'; 
	static uint8_t is_triggering = 0;
	static uint8_t is_triggering_ch = 0;
	static float trigger_time = 0;

	if( inputAvailable() || auto_test ) {
		if(auto_test) {
			if(!is_triggering) auto_char++;
			if(auto_char > '6') auto_char = 'q';
			input = auto_char;
		} else  {
			input = getchar();
		}

		if(input > 0) {
			switch(input) {

				case 't':
					display_telemetry? display_telemetry=false: display_telemetry=true;
					break;

				case 'T':
					if(!send_actuators) send_actuators = true;
					if(!is_triggering) {
						is_triggering = 1;
						is_triggering_ch = 15;
						trigger_time = getElapsedTime();
					}
					break;

				case 'A':
				case 'B':
				case 'C':
				case 'D':
				case 'E':
				case 'F':
					input = input-'A'+1+'9';
				case '0':
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
				case '7':
				case '8':
				case '9':
					if(!send_actuators) send_actuators = true;
					if(!is_triggering) {
						printf("Triggerging channel %u\n",input - '0');
						is_triggering = 1;
						is_triggering_ch = input - '0';
						trigger_time = getElapsedTime();
					}
					break;

				case 'f':
					{
						char path[256];
						if(strlen(log_filename)) {
							strcpy(path, log_filename);
						} else {
							// restore terminal for line input
							restoreTerminal();
							printf("Enter log file path: ");
							fflush(stdout);
							if(fgets(path, sizeof(path), stdin)) {
								char *nl = strchr(path, '\n');
								if(nl) *nl = '\0';
							}
							initTerminal();
						}
						if(strlen(path)) {
							if(!runLogReplay(path)) {
								return;
							}
							printTestHelp();
						}
					}
					break;

				case 'p':
					printTestHelp();
					break;

				case 3: // <CTRL-C>
					// allow flowthrough
				case 'q':
					printf("Keyboard caught exit signal ...\n");
					zeroAcutators();
					running = false;
					break;

				default:
					break;
			}
			input = 0;
		} else {
			clearerr(stdin);
		}
	}

	for(uint8_t i=0; i<16; i++) {
		if( (actuator_types[i] == ACT_L_THROTTLE) ||
				(actuator_types[i] == ACT_R_THROTTLE) ||
				(actuator_types[i] == ACT_ROTOR) ) {
			if(is_triggering && (is_triggering_ch) == i) {
				actuators[i] = 1100;
			} else {
				actuators[i] = 1000;
			}
		} else {
			if(is_triggering && (is_triggering_ch) == i) {
					actuators[i] = 1700;
			} else {
					actuators[i] = 1500;
			}
		}
	}
	if(getElapsedTime() - trigger_time > TRIGGER_LENGTH) {
		is_triggering = 0;
		is_triggering_ch = 15;
	}


	if(print_timing) {
	printf("LLA %04.1f  UTC %04.01f  VEL %04.01f  HS %04.01f | MAG %05.01f | STAT %05.01f\n",
			(float)gnss_lla_cnt/getElapsedTime(),
			(float)gnss_utc_cnt/getElapsedTime(),
			(float)gnss_vel_cnt/getElapsedTime(),
			(float)gnss_hs_cnt/getElapsedTime(),
			(float)mag_cnt/getElapsedTime(),
			(float)stat_p_cnt/getElapsedTime()
			);
	}

	if(display_telemetry) {
		for(uint8_t i=0; i<16; i++)
			printf("%04u ",actuators[i]);
		printf(" [%u] \n", is_triggering_ch);
	}

	if(send_actuators)
		BRIDGE_SendActuatorPkt(1,actuators);

}

uint16_t commConstruct(uint8_t type, PacketAction_t action, void * data, uint16_t size, const void * parameter, bool uses_address, Packet * packet) { 
	packet->clear();

	if(uses_address) {
		packet->setAddressing(true);
		packet->setFromAddress(ALL_NODES);
		packet->setToAddress(ALL_NODES); // FIXME - should find real address
	} else {
		packet->setAddressing(false);
	}

	packet->setType(type);
	packet->setAction(action);
	packet->setData((uint8_t *)data, size);

	return 0;
}

uint16_t commWrite(uint8_t type, PacketAction_t action, void * data, uint16_t size, const void * parameter) {
	uint16_t tx_size = 0, retval = 0;

	if((type&0xF0) != 0x60 && (type <= 0xE8 || type >= 0xEF) && type != PAYLOAD_S0_SENSORS) {
		if((cmd_buf_end + 1) % CMD_BUF_SIZE == cmd_buf_start) {
			pmesg(VERBOSE_ERROR,"Command Buffer Overflow!\n");
			retval = 0;
		} else {
			commConstruct(type, action, data, size, parameter, true, 
					&cmd_buf[cmd_buf_end]);
			retval = cmd_buf[cmd_buf_end].getSize();

			cmd_buf_end =  (cmd_buf_end + 1) % CMD_BUF_SIZE;
		}

	} else {

		if((pkt_buf_end + 1) % PKT_BUF_SIZE == pkt_buf_start) {
			pmesg(VERBOSE_ERROR,"Packet Buffer Overflow!\n");
			retval = 0;
		} else {
			commConstruct(type, action, data, size, parameter, true, 
					&pkt_buf[pkt_buf_end]);
			retval = pkt_buf[pkt_buf_end].getSize();

			pkt_buf_end =  (pkt_buf_end + 1) % PKT_BUF_SIZE;
		}
	}

	while(cmd_buf_start != cmd_buf_end) {
		tx_size = comm_interface->write(cmd_buf[cmd_buf_start].getPacket(), cmd_buf[cmd_buf_start].getSize(), 0x5300);
		if(tx_size == cmd_buf[cmd_buf_start].getSize()) {
			cmd_buf_start = (cmd_buf_start + 1) % CMD_BUF_SIZE;
		} else {
			return retval;
		}
	}

	while(pkt_buf_start != pkt_buf_end) {
		tx_size = comm_interface->write(pkt_buf[pkt_buf_start].getPacket(), pkt_buf[pkt_buf_start].getSize(), 0x5300);
		if(tx_size == pkt_buf[pkt_buf_start].getSize()) {
			pkt_buf_start = (pkt_buf_start + 1) % PKT_BUF_SIZE;
		} else {
			return retval;
		}
	}

	return retval;
}
