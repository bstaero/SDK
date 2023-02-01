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
#include <termios.h>

#include "test.h"
#include "main.h"
#include "structs.h"

#include "flight_plan.h"

#include "bridge.h"


// variables
volatile bool display_telemetry = false;
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

extern uint32_t gnss_lla_cnt;
extern uint32_t gnss_utc_cnt;
extern uint32_t gnss_vel_cnt;
extern uint32_t gnss_hs_cnt;

extern uint32_t mag_cnt;

extern uint32_t stat_p_cnt;

CAN_DeploymentTube_t deployment_tube;
uint8_t new_deployment_tube_data = 0;

// functional definitions
void SendState(CAN_DeploymentTubeState_t state);
void SendHeartbeat();

// packet for transmision
Packet              tx_packet;

// for command line (terminal) input
struct termios initial_settings, new_settings;

void printTestHelp() {
	printf("Keys:\n");
	printf("  t   : Toggle telemetry display\n");
	printf("  i   : Toggle timing display\n");
	printf("\n");
	printf("  T   : take a picture\n");
	printf("\n");
	printf("  h   : Toggle heartbeat\n");
	printf("  r   : Set state ready\n");
	printf("  a   : Set state armed\n");
	printf("  A   : Emergency aircraft release\n");
	printf("\n");
	printf("  p   : print this help\n");
}

bool inputAvailable()  
{
	// check for input on terminal
	struct timeval tv;
	fd_set fds;
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds);
	select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);

	return (FD_ISSET(0, &fds));
}


void initializeTest() {

	// terminal settings to get input
	tcgetattr(0,&initial_settings);

	new_settings = initial_settings;
	new_settings.c_lflag &= ~ICANON;
	new_settings.c_lflag &= ~ECHO;
	new_settings.c_lflag &= ~ISIG;
	new_settings.c_cc[VMIN] = 0;
	new_settings.c_cc[VTIME] = 0;

	tcsetattr(0, TCSANOW, &new_settings);
}


void updateTest() {
	char input; 
	static uint8_t is_triggering = 0;
	static float trigger_time = 0;

	static uint8_t sending_heartbeat = 0;
	static float last_heartbeat = 0;

	if(last_heartbeat == 0) last_heartbeat = getElapsedTime();

	if(sending_heartbeat && (getElapsedTime() - last_heartbeat > 1.0)) {
		SendHeartbeat();
		last_heartbeat = getElapsedTime();
	}

	if( inputAvailable() ) {
		input = getchar();

		if(input > 0) {
			switch(input) {

				case 't':
					display_telemetry? display_telemetry=false: display_telemetry=true;
					break;

				case 'i':
					print_timing = !print_timing;
					break;

				case 'T':
					if(!is_triggering) {
						is_triggering = 1;
						trigger_time = getElapsedTime();
					}
					break;

				case 'h':
					if(!sending_heartbeat)
						sending_heartbeat = 1;
					else
						sending_heartbeat = 0;
					break;

				case 'r':
					printf("Requesting state READY\n");
					SendState(DEPLOY_TUBE_READY);
					break;

				case 'A':
					printf("Requesting emergency aircraft release\n");
					SendState(DEPLOY_TUBE_AC_RELASED);
					break;

				case 'a':
					printf("Requesting state ARMED\n");
					SendState(DEPLOY_TUBE_ARMED);
					break;

				case 'p':
					printTestHelp();
					break;

				case 3: // <CTRL-C> 
					// allow flowthrough
				case 'q':
					printf("Keyboard caught exit signal ...\n");
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

	uint16_t actuators[16];

	if(is_triggering) {
		if(getElapsedTime() - trigger_time > 0.1) is_triggering = 0;
		actuators[14] = 2000;
	} else {
		actuators[14] = 1000;
	}

	if(display_telemetry) {
		if(new_deployment_tube_data) {
			new_deployment_tube_data = 0;
			// DEBUG - sanity check
			char state[8];

			switch(deployment_tube.state) {
				case DEPLOY_TUBE_INIT:          sprintf(state, "INIT   "); break;
				case DEPLOY_TUBE_READY:         sprintf(state, "READY  "); break;
				case DEPLOY_TUBE_ARMED:         sprintf(state, "ARMED  "); break;
				case DEPLOY_TUBE_FLAP_OPEN:     sprintf(state, "FL OPEN"); break;
				case DEPLOY_TUBE_PARA_DEPLOYED: sprintf(state, "PARA DP"); break;
				case DEPLOY_TUBE_JETTISONED:    sprintf(state, "TUB JET"); break;
				case DEPLOY_TUBE_AC_RELASED:    sprintf(state, "AC REL "); break;
				case DEPLOY_TUBE_ERROR:         sprintf(state, "ERROR  "); break;
			}

			char door[8];

			if(deployment_tube.parachute_door) sprintf(door,"OPEN  ");
			else sprintf(door,"CLOSED");

			printf("DEPLOY TUBE: %0.02f s, state %s door %s error 0x%08x\n", 
					getElapsedTime(), state, door, deployment_tube.error);
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
	}

}

void sendOverCAN(uint32_t id, void * data, uint8_t size) {
	((char *)data)[0] = '@';
	setFletcher16((uint8_t *)data, size);	

	uint8_t can_tx_buffer[500];

	uint8_t ptr = 0;

	memcpy(can_tx_buffer + ptr,&id,4); ptr += 4;
	memcpy(can_tx_buffer + ptr,&size,1); ptr += 1;
	memcpy(can_tx_buffer + ptr,data,size); ptr += size;

	tx_packet.setAddressing(false);
	tx_packet.setType(HWIL_CAN);
	tx_packet.setAction(PKT_ACTION_STATUS);
	tx_packet.setData(can_tx_buffer, ptr);

	writeBytes(tx_packet.getPacket(),tx_packet.getSize());
}

void SendState(CAN_DeploymentTubeState_t state) {
		CAN_DeploymentTubeCommand_t data;

		data.id = CMD_SET_STATE;
		data.value = (float)state;

		uint32_t id = CAN_PKT_DEPLOYMENT_TUBE_CMD;
		uint8_t size = sizeof(CAN_DeploymentTubeCommand_t);

		sendOverCAN(id,&data,size);
}

void SendHeartbeat() {
	CAN_DeploymentTubeCommand_t data;

	data.id = CMD_HEARTBEAT;
	data.value = 0;

	uint32_t id = CAN_PKT_DEPLOYMENT_TUBE_CMD;
	uint8_t size = sizeof(CAN_DeploymentTubeCommand_t);

	sendOverCAN(id,&data,size);
}

void exitTest() {
	tcsetattr(0, TCSANOW, &initial_settings);
}
