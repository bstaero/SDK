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
#include "example_common.h"
#include "structs.h"

#include "flight_plan.h"

#include "bridge.h"


// variables
volatile bool display_telemetry = false;
volatile bool write_file = false;

static bool send_actuators = false;
static bool print_timing = false;
static float last_print_time = 0;

extern uint32_t stat_p_cnt;

static uint8_t sending_heartbeat = 1;
static uint8_t sending_flight_mode = 0;

static float last_actuators = 0;

#define TRIGGER_LENGTH 1.0


CAN_AirData_t local_air_data;
CAN_Supply_t local_supply;

CAN_DeploymentTube_t deployment_tube;
uint8_t new_deployment_tube_data = 0;


volatile CAN_SensorType_t calibration_requested = CAN_UNKNOWN_SENSOR;

volatile bool waiting_on_calibrate = false;
void updateCalibration(void);


#define CAN_COMMAND_TIMEOUT 2.0

int8_t local_engine_kill = 1; // start killed
int8_t remote_engine_kill = -1;

void updateEngineKill(void);

int8_t local_ap_enable = 1; // start on
int8_t remote_ap_enable = 1;

void updateAPEnable(void);

// packet for transmision
Packet              tx_packet;

void printTestHelp() {
	printf("Keys:\n");
	printf("  t   : Toggle telemetry display\n");
	printf("  i   : Toggle timing display\n");
	printf("\n");
	printf("  1 to A   : test channel n\n");
	printf("\n");
	printf("  h   : Toggle heartbeat\n");
	printf("  r   : Set state ready\n");
	printf("  a   : Set state armed\n");
	printf("  !   : Emergency aircraft release\n");
	printf("  s   : Send shutdown to deployment tube\n");
	printf("\n");
	printf("  k   : Toggle engine enable\n");
	printf("  S   : Send shutdown to ap\n");
	printf("  H   : Command humidity recondition\n");
	printf("\n");
	printf("  f   : Toggle flight mode heartbeat\n");
	printf("\n");
	printf("  p   : print this help\n");
}

void updateTest() {
	char input; 
	static uint8_t is_triggering = 0;
	static float trigger_time = 0;
	static uint16_t actuators[16];

	static float last_heartbeat = 0;

	static float last_flight_mode = 0;
	static uint8_t cleared_flight_mode = 1;

	if(last_heartbeat == 0) last_heartbeat = getElapsedTime();

	if(sending_heartbeat && (getElapsedTime() - last_heartbeat > 0.5)) {
		BRIDGE_SendDeployTubeCmdPkt(1, CMD_HEARTBEAT, 0);
		last_heartbeat = getElapsedTime();
	}

	if(last_flight_mode == 0) last_flight_mode = getElapsedTime();

	if(sending_flight_mode && (getElapsedTime() - last_flight_mode > 0.5)) {
		cleared_flight_mode = 0;
		BRIDGE_SendCommandPkt(1, CMD_FLIGHT_MODE, FLIGHT_MODE_FLYING);
		last_flight_mode = getElapsedTime();
	}

	if(!sending_flight_mode && !cleared_flight_mode) {
		cleared_flight_mode = 1;
		BRIDGE_SendCommandPkt(1, CMD_FLIGHT_MODE, FLIGHT_MODE_PREFLIGHT);
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
					if(print_timing) {
						stat_p_cnt = 0;

						last_print_time = getElapsedTime();
					}
					break;

					
				case 'A':
				case 'B':
				case 'C':
				case 'D':
				case 'E':
				case 'F':
					input = input-'A'+1+'9';
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
						//printf("Triggerging channel %u\n",input - '0');
						is_triggering = input - '0';
						trigger_time = getElapsedTime();
					}
					break;


				case 'h':
					if(!sending_heartbeat)
						sending_heartbeat = 1;
					else
						sending_heartbeat = 0;

					if(sending_heartbeat)	printf("Sending heartbeat\n");
					else	printf("Paused heartbeat\n");
					break;

				case 'r':
					printf("Requesting state READY\n");
					BRIDGE_SendDeployTubeCmdPkt(1, CMD_SET_STATE, (float)DEPLOY_TUBE_READY);
					break;

				case '!':
					printf("Requesting emergency aircraft release\n");
					BRIDGE_SendDeployTubeCmdPkt(1, CMD_SET_STATE, (float)DEPLOY_TUBE_AC_RELASED);
					break;

				case 'a':
					printf("Requesting state ARMED\n");
					BRIDGE_SendDeployTubeCmdPkt(1, CMD_SET_STATE, (float)DEPLOY_TUBE_ARMED);
					break;


				case 'H':
					if(calibration_requested == CAN_UNKNOWN_SENSOR) {
						BRIDGE_SendCalibratePkt(1, CAN_HUMIDITY, CAN_REQUESTED);

						calibration_requested = CAN_HUMIDITY;
						waiting_on_calibrate = true;
						printf("Humidity Recondition Requested.. ");
						fflush(stdout);
					}
					break;

				case 'f':
					if(!sending_flight_mode)
						sending_flight_mode = 1;
					else
						sending_flight_mode = 0;

					if(sending_flight_mode)	printf("Sending Flight Mode Flying\n");
					else	printf("Paused Flight Mode\n");
					break;
					
				case 'S':
					local_ap_enable = 0;
					
					remote_engine_kill = local_engine_kill;

					break;

				case 'k':
					if(local_engine_kill == 1)
						local_engine_kill = 0;
					else
						local_engine_kill = 1;

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

	for(uint8_t i=0; i<16; i++) {
		if(is_triggering && (is_triggering-1) == i) {
			if(i == 2 || i == 3)
				actuators[i] = 1050;
			else
				actuators[i] = 1800;
		} else {
			actuators[i] = 1000;
		}
	}
	if(getElapsedTime() - trigger_time > TRIGGER_LENGTH) is_triggering = 0;

	if(send_actuators && getElapsedTime() - last_actuators > 0.02) {
		last_actuators = getElapsedTime();
		BRIDGE_SendActuatorPkt(1, actuators);
	}

	if(waiting_on_calibrate) updateCalibration();

	updateEngineKill();

	updateAPEnable();

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
				case DEPLOY_TUBE_SHUTDOWN:      sprintf(state, "SHTDWN "); break;
				case DEPLOY_TUBE_ERROR:         sprintf(state, "ERROR  "); break;
			}

			char door[8];

			if(deployment_tube.parachute_door) sprintf(door,"OPEN  ");
			else sprintf(door,"CLOSED");

			printf("%s door %s %0.1fV 0x%08x ", 
					state, door, (float)deployment_tube.batt_voltage / 10.f, deployment_tube.error);

			for(uint8_t i=0; i<16; i++)
				printf("%04u ",actuators[i]);

			printf(" %+07.01f Pa %+05.01f deg C %04.01f %%  ",
					local_air_data.static_pressure,
					local_air_data.air_temperature,
					local_air_data.humidity);

			printf("%.01f V, %.01f A, %.01f mAh, %.01f deg C  ",
					local_supply.voltage,
					local_supply.current,
					local_supply.coulomb_count,
					local_supply.temperature);

			printf("\n");
		}

	} else {
		if(print_timing && getElapsedTime() - last_print_time > 1.0) {
			printf("SONDE %04.1f \n",
					(float)stat_p_cnt/(getElapsedTime() - last_print_time)
					);
			stat_p_cnt = 0;
			last_print_time = getElapsedTime();
		}
	}

}

void updateCalibration(CAN_SensorType_t sensor,
		CAN_CalibrationState_t state) {

	if(state == CAN_CALIBRATED)
		if(sensor == (CAN_SensorType_t)calibration_requested)
			calibration_requested = CAN_UNKNOWN_SENSOR;
}

void updateCalibration() {
	static float end_time = 0.0;
	if(end_time == 0.0 && calibration_requested != CAN_UNKNOWN_SENSOR) {
		switch(calibration_requested) {
			case CAN_DYNAMIC_PRESSURE: end_time = getElapsedTime() + 2.0; break;
			case CAN_GYROSCOPE:        end_time = getElapsedTime() + 2.0; break;
			case CAN_MAGNETOMETER:     end_time = getElapsedTime() + 60.0; break;
			case CAN_HUMIDITY:         end_time = getElapsedTime() + 260.0; break;
		}
	}

	if(calibration_requested != CAN_UNKNOWN_SENSOR && getElapsedTime() < end_time)
		return;

	if(getElapsedTime() < end_time) {
		printf("SUCCESS\n");
	} else {
		calibration_requested = CAN_UNKNOWN_SENSOR;
		printf("FAILED\n");
	}

	end_time = 0.0;
	waiting_on_calibrate = false;
}

void updateEngineKill() {
	static float end_time = 0.0;

	if(local_engine_kill != remote_engine_kill) {
		if(end_time == 0.0 ) {
			end_time = getElapsedTime() + CAN_COMMAND_TIMEOUT;

			BRIDGE_SendCommandPkt(1, CMD_ENGINE_KILL, (float)local_engine_kill);

			if(local_engine_kill)
				printf("Engine Kill Sent.. ");
			else
				printf("Engine Enable Sent.. ");
			fflush(stdout);
		}
	}

	if(end_time > 0.0) {
		if(getElapsedTime() > end_time) {
			printf("TIMED OUT \n");
			end_time = 0.0;
		} else {
			if(remote_engine_kill == local_engine_kill) {
				end_time = 0.0;
				printf("SUCCESS \n");
			}
		}
	}

}

void updateAPEnable() {
	static float end_time = 0.0;

	if(local_ap_enable != remote_ap_enable && local_ap_enable == 0) {
		if(end_time == 0.0 ) {
			end_time = getElapsedTime() + CAN_COMMAND_TIMEOUT;

			BRIDGE_SendCommandPkt(1, CMD_DOWNLOAD_LOG, 2.0);

			if(!local_ap_enable)
				printf("AP Commanded to Shut Down.. ");
			fflush(stdout);

			local_ap_enable = 1;  // FIXME - need to confirm with response
		}
	} 

	if(end_time > 0.0) {
		if(getElapsedTime() > end_time) {
			printf("TIMED OUT \n");
			end_time = 0.0;
		} else {
			if(remote_ap_enable == local_ap_enable) {
				end_time = 0.0;
				printf("SUCCESS \n");
			}
		}
	}

}

void simuatedShutdown(void) {
	printf("Shutdown requested\n");
	BRIDGE_SendCommandPkt(1,CMD_DOWNLOAD_LOG, 2.0);

	// Stop sending packets
	send_actuators = false;
	sending_heartbeat = 0;
	sending_flight_mode = 0;
}

