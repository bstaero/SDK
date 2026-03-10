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

#include "test_handler.h"

// variables
bool show_telemetry = false;

// functional definitions
bool sendPayloadData(uint8_t * data, uint8_t size);

// packet for transmision
Packet              tx_packet;


#define CMD_BUF_SIZE 8
Packet cmd_buf[CMD_BUF_SIZE];
uint8_t cmd_buf_start = 0;
uint8_t cmd_buf_end = 0;

#define PKT_BUF_SIZE 8
Packet pkt_buf[PKT_BUF_SIZE];
uint8_t pkt_buf_start = 0;
uint8_t pkt_buf_end = 0;

extern TelemetryOrientation_t telemetry_orientation;
extern TelemetryPosition_t    telemetry_position;
extern TelemetryPressure_t    telemetry_pressure;
extern TelemetrySystem_t      telemetry_system;
extern TelemetryControl_t     telemetry_control;

extern UserPayload_t          rx_payload;
UserPayload_t                 tx_payload;

FlightPlanMap_t flight_plan_map;
FlightPlan flight_plan;

void printTestHelp() {
	printf("Keys:\n");
	printf("  c   : Send payload control data\n");
	printf("  t   : Toggle Telemetry Display\n");
	printf("\n");
	printf("  p   : print this help\n");
}

extern DeploymentTube_t deployment_tube;

bool new_telemetry_system = false;

void runDeploymentTest() {

	static bool show_state = true;

	if(show_state) {
		show_state = false;
	char dt_state[8];

	switch(deployment_tube.state) {
		case DEPLOY_TUBE_INIT:          sprintf(dt_state, "INIT   "); break;
		case DEPLOY_TUBE_READY:         sprintf(dt_state, "READY  "); break;
		case DEPLOY_TUBE_ARMED:         sprintf(dt_state, "ARMED  "); break;
		case DEPLOY_TUBE_FLAP_OPEN:     sprintf(dt_state, "FL OPEN"); break;
		case DEPLOY_TUBE_PARA_DEPLOYED: sprintf(dt_state, "PARA DP"); break;
		case DEPLOY_TUBE_JETTISONED:    sprintf(dt_state, "TUB JET"); break;
		case DEPLOY_TUBE_AC_RELASED:    sprintf(dt_state, "AC REL "); break;
		case DEPLOY_TUBE_SHUTDOWN:      sprintf(dt_state, "SHTDWN "); break;
		case DEPLOY_TUBE_ERROR:         sprintf(dt_state, "ERROR  "); break;
	}

	char fl_state[8];

	switch(telemetry_system.flight_mode) {
		case FLIGHT_MODE_INIT:                  sprintf(fl_state, "INIT   "); break;
		case FLIGHT_MODE_PREFLIGHT:             sprintf(fl_state, "PREFL  "); break;
		case FLIGHT_MODE_CALIBRATE:             sprintf(fl_state, "CALIB  "); break;
		case FLIGHT_MODE_LAUNCH:                sprintf(fl_state, "LAUNCH "); break;
		case FLIGHT_MODE_CLIMBOUT:              sprintf(fl_state, "CLIMB  "); break;
		case FLIGHT_MODE_TRANSITION_TO_FORWARD: sprintf(fl_state, "TRANS F"); break;
		case FLIGHT_MODE_FLYING:                sprintf(fl_state, "FLYING "); break;
		case FLIGHT_MODE_TRANSITION_TO_HOVER:   sprintf(fl_state, "TRANS H"); break;
		case FLIGHT_MODE_LANDING:               sprintf(fl_state, "LANDING"); break;
		case FLIGHT_MODE_LANDED:                sprintf(fl_state, "LANDED "); break;
		case FLIGHT_MODE_POSTFLIGHT:            sprintf(fl_state, "POST FL"); break;
		case FLIGHT_MODE_TERMINATE:             sprintf(fl_state, "TERM   "); break;
		case FLIGHT_MODE_INVALID_MODE:          sprintf(fl_state, "INVALID"); break;
	}

	printf("pdop %04.1f DT Mode: %s Flight Mode: %s Engine: %u\n",
			telemetry_system.pdop/100.f,
			dt_state,
			fl_state,
			telemetry_system.engine_on);
	}

	static uint8_t test_state = 0;
	static float last_message = 0;

	float now = getElapsedTime();
	static float CMD_TIMEOUT = 4.0;

	static bool first_run = false;

	static float flight_time = -1.0;

	switch(test_state) {

		case 0:

			if(telemetry_system.flight_mode == FLIGHT_MODE_PREFLIGHT) {
				if(first_run) {
					printf("!!!!!!!!!!!!!!!!!!!!!!! REBOOT DETECTED\n'");
					running = false;
				}
				printf("--> Preflight mode set\n");
				show_state = true;
				test_state ++;
				first_run = true;
			}

			if(first_run) {
				first_run = false;
			}

			if(new_telemetry_system) {
				new_telemetry_system = false;
				printf("--> Requesting preflight mode\n");

				setCommandValue(CMD_FLIGHT_MODE, FLIGHT_MODE_PREFLIGHT, false);
				usleep(1000);
				setCommandValue(CMD_FLIGHT_MODE, FLIGHT_MODE_PREFLIGHT, false);
			}

			break;

		case 1:
			show_state = true;
			if(telemetry_system.pdop > 0 && telemetry_system.pdop < 500) {
				printf("--> PDOP good\n");
				test_state ++;
				first_run = true;
			}
			break;

		case 2:
			if(telemetry_system.flight_mode == FLIGHT_MODE_LAUNCH) {
				printf("--> Launch mode set\n");
				show_state = true;
				test_state ++;
				first_run = true;
				break;
			}

			if(new_telemetry_system) {
				new_telemetry_system = false;
				printf("--> Requesting launch mode\n");

				setCommandValue(CMD_FLIGHT_MODE, FLIGHT_MODE_LAUNCH, false);
				usleep(1000);
				setCommandValue(CMD_FLIGHT_MODE, FLIGHT_MODE_LAUNCH, false);
			}

			break;

		case 3:
			if(telemetry_system.engine_on == 1 ||
					deployment_tube.state  == DEPLOY_TUBE_FLAP_OPEN ||
					deployment_tube.state  == DEPLOY_TUBE_PARA_DEPLOYED ||
					deployment_tube.state  == DEPLOY_TUBE_JETTISONED ||
					deployment_tube.state  == DEPLOY_TUBE_AC_RELASED
					) {
				printf("--> Engine enabled\n");
				show_state = true;
				test_state ++;
				first_run = true;
				break;
			}

			if(new_telemetry_system) {
				new_telemetry_system = false;
				printf("--> Requesting engine enable\n");

				setCommandValue(CMD_ENGINE_KILL, 0, false);
				usleep(1000);
				setCommandValue(CMD_ENGINE_KILL, 0, false);
			}

			break;

		case 4:
			show_state = true;
			if(deployment_tube.state == DEPLOY_TUBE_AC_RELASED) {
				printf("--> Aricraft Released\n");
				test_state ++;
				first_run = true;
				break;
			}
			if(telemetry_system.flight_mode == FLIGHT_MODE_PREFLIGHT) {
				printf("!!!!!!!!!!!!!!!!!!!!!!! REBOOT DETECTED\n'");
				running = false;
			}
			break;

		case 5:

			if(telemetry_system.flight_mode != FLIGHT_MODE_LAUNCH ) {
				printf("--> Flight mode set\n");
				show_state = true;
				test_state ++;
				first_run = true;
				break;
			}

			if(first_run) {
				if(telemetry_system.flight_mode == FLIGHT_MODE_PREFLIGHT) {
					printf("!!!!!!!!!!!!!!!!!!!!!!! REBOOT DETECTED\n'");
					running = false;
				}
			}

			if(first_run) {
				first_run = false;
			}

			if(new_telemetry_system) {
				new_telemetry_system = false;
				printf("--> Requesting flight mode\n");

				setCommandValue(CMD_FLIGHT_MODE, FLIGHT_MODE_FLYING, false);
				usleep(1000);
				setCommandValue(CMD_FLIGHT_MODE, FLIGHT_MODE_FLYING, false);
			}

			break;

		case 6:

			if(telemetry_system.engine_on == 0) {
				printf("--> Engine disabled\n");
				flight_time = -1.0;
				show_state = true;
				test_state ++;
				first_run = true;
				break;
			}

			if(first_run) {
				first_run = false;
				flight_time = now;
			}

			if(now-flight_time > 5.0) {
				if(new_telemetry_system) {
					new_telemetry_system = false;
					printf("\n--> Requesting engine disable\n");

					setCommandValue(CMD_ENGINE_KILL, 1.0, false);
					usleep(1000);
					setCommandValue(CMD_ENGINE_KILL, 1.0, false);
				}
			} else {
				printf("Shutdown in %04.1fs\r",5.0-(now-flight_time));
				fflush(stdout);
			}

			break;

		default:
			test_state = 0;
			first_run = true;
			break;
	}
}

void updateTest() {
	char input; 

	Command_t command;

	uint8_t num_points = 0;
	Waypoint_t temp_waypoint;
	Waypoint_t temp_waypoints[MAX_WAYPOINTS];


	runDeploymentTest();

	if( inputAvailable() ) {
		input = getchar();

		if(input > 0) {
			switch(input) {
				case 'c':
					printf("Sending payload data\n");

					uint8_t data[128];
					sprintf((char*)data,"This is a test %07.01f\n\r",getElapsedTime());
					sendPayloadData(data,strlen((char*)data));
					break;

				case 'r':

					// change to preflight mode if we need to
					if (telemetry_system.flight_mode == FLIGHT_MODE_INVALID_MODE ||
							telemetry_system.flight_mode == FLIGHT_MODE_LANDED       ||
							telemetry_system.flight_mode == FLIGHT_MODE_LAUNCH) {
						if (!setCheckCommandValue(CMD_FLIGHT_MODE, FLIGHT_MODE_PREFLIGHT, (uint8_t*)&telemetry_system.flight_mode, FLIGHT_MODE_PREFLIGHT, 4.0)) {
							printf(" Command to change to PREFLIGHT mode failed\n");
							break;
						}
					}

					if (telemetry_system.flight_mode == FLIGHT_MODE_PREFLIGHT || telemetry_system.flight_mode == FLIGHT_MODE_LAUNCH) {
						// change to launch mode from preflight mode 
						if (telemetry_system.flight_mode == FLIGHT_MODE_PREFLIGHT) {

							printf(" Switching to Launch Mode ...");
							if (!setCheckCommandValue(CMD_FLIGHT_MODE, FLIGHT_MODE_LAUNCH, (uint8_t*)&telemetry_system.flight_mode, FLIGHT_MODE_LAUNCH, 4.0)) {
								printf(" failed\n");
								break;
							} else
								printf(" done\n");
						}
					}

					// Fall through
					//printf("NOT IN VALID FLIGHT MODE TO LAUNCH\n");

					break;

				case 'k':
						// enable engine and launch 
						//if (telemetry_system.flight_mode == FLIGHT_MODE_LAUNCH) {
							printf(" Disabling Engine ...");
							if (!setCheckCommandValue(CMD_ENGINE_KILL, 1, (uint8_t*)&telemetry_system.engine_on, 1, 4.0)) {
								printf(" failed\n");
								break;
							}
							else printf(" done\n");

							/*printf(" Launching ...");
							if (!setCheckCommandValue(CMD_LAUNCH, 0, (uint8_t*)&telemetry_system.flight_mode, FLIGHT_MODE_CLIMBOUT, 1.0)) {
								printf(" failed\n");
							}
							else printf(" launched\n");*/
			//}

							break;

				case 'e':
						// enable engine and launch 
						//if (telemetry_system.flight_mode == FLIGHT_MODE_LAUNCH) {
							printf(" Enabling Engine ...");
							if (!setCheckCommandValue(CMD_ENGINE_KILL, 0, (uint8_t*)&telemetry_system.engine_on, 1, 4.0)) {
								printf(" failed\n");
								break;
							}
							else printf(" done\n");

							/*printf(" Launching ...");
							if (!setCheckCommandValue(CMD_LAUNCH, 0, (uint8_t*)&telemetry_system.flight_mode, FLIGHT_MODE_CLIMBOUT, 1.0)) {
								printf(" failed\n");
							}
							else printf(" launched\n");*/
			//}

							break;

				case 't':
					show_telemetry = !show_telemetry;
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

	//if(show_telemetry)
}

bool sendPayloadData(uint8_t * data, uint8_t size) {
	uint8_t ptr = 0;

	while(ptr < size) {
		if(size-ptr > 64) tx_payload.size = 64;
		else tx_payload.size = size-ptr;

		memcpy(tx_payload.buffer,&(data[ptr]),tx_payload.size);

		comm_handler->send(PAYLOAD_DATA_CHANNEL_0, (uint8_t *)&tx_payload, sizeof(UserPayload_t), NULL);

		ptr += tx_payload.size;
		tx_payload.size = 0;
		for(uint8_t i=0; i<64; i++) tx_payload.buffer[i] = 0;
	}

	return true;
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
