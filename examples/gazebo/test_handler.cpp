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
#define _USE_MATH_DEFINES

#include <stdio.h>
#include <stdlib.h>
#include <cmath>

#include <algorithm>
#include <numeric>
#include <vector>

#include "test.h"
#include "main.h"
#include "structs.h"

// Payload state
extern SystemInitialize_t  system_init;

extern volatile bool received_reply;
extern Command_t     set_command;
extern volatile bool set_command_ack;

extern volatile bool show_telemetry;

extern PayloadControl_t    payload_current_state;

TelemetryPosition_t telemetry_position;
TelemetrySystem_t   telemetry_system;
TelemetryControl_t  telemetry_control;

CalibrateSensor_t * calibration_data;

State_t estimator_data;
SingleValueSensor_t agl_data;
DubinsPath_t dubins_path;

volatile SensorType_t calibration_requested = UNKNOWN_SENSOR;


void receive(uint8_t type, void * data, uint16_t size, const void * parameter)
{
	switch(type) {
		/* SENSORS */
		case SENSORS_GPS:
		case SENSORS_ACCELEROMETER:
		case SENSORS_GYROSCOPE:
		case SENSORS_MAGNETOMETER:
		case SENSORS_IMU:
		case SENSORS_DYNAMIC_PRESSURE:
		case SENSORS_STATIC_PRESSURE:
		case SENSORS_AIR_TEMPERATURE:
		case SENSORS_AGL:
			memcpy(&agl_data,data,sizeof(SingleValueSensor_t));
			if(show_telemetry)
				printf("\tLaser:\t:0.02f\n",agl_data.value);
			break;

		case SENSORS_CALIBRATE:
			calibration_data = (CalibrateSensor_t *)data;
			if(calibration_data->state == CALIBRATED)
				if(calibration_data->sensor == calibration_requested) {
					printf(" Success: Sensor calibrated\n");
					calibration_requested = UNKNOWN_SENSOR;
				}
			break;

#ifdef VEHICLE_FIXEDWING
		case SENSORS_OFFSETS:
		case SENSORS_ORIENTATION:
#endif

			/* STATE */
		case STATE_STATE:
			break;

		case STATE_ESTIMATOR_PARAM:
			if(show_telemetry)
				printf("STATE_ESTIMATOR_PARAM\n");
			memcpy(&estimator_data,data,sizeof(State_t));
			break;

			/* CONTROL */
		case CONTROL_COMMAND:
		case CONTROL_PID:
		case CONTROL_FLIGHT_PARAMS:
		case CONTROL_FILTER_PARAMS:

		case DUBIN_PATH:
			memcpy(&dubins_path,data,sizeof(DubinsPath_t));
			printf("DUBIN_PATH\n");
			printf("  xc0 <%0.02f %0.02f %0.02f> xc2 <%0.02f %0.02f %0.02f>\n"
					   "  t0 [%0.02f,%0.02f] t1 [%0.02f,%0.02f] t2 [%0.02f,%0.02f]\n"
					   "  a %0.02f b %0.02f c [%0.02f %0.02f %0.02f]\n"
					   "  x0 %0.02f y0 %0.02f z0 [%0.02f %0.02f %0.02f]\n"
					   "  R %0.02f (%0.02f %0.02f)\n"
					   "  %u\n",
						 dubins_path.xc0_x,
						 dubins_path.xc0_y,
						 dubins_path.xc1_x,
						 dubins_path.xc1_y,
						 dubins_path.xc2_x,
						 dubins_path.xc2_y,
						 dubins_path.t0_i,
						 dubins_path.t0_f,
						 dubins_path.t1_i,
						 dubins_path.t1_f,
						 dubins_path.t2_i,
						 dubins_path.t2_f,
						 dubins_path.a,
						 dubins_path.b,
						 dubins_path.c0,
						 dubins_path.c1,
						 dubins_path.c2,
						 dubins_path.x0,
						 dubins_path.y0,
						 dubins_path.z00,
						 dubins_path.z01,
						 dubins_path.z02,
						 dubins_path.big_r,
						 dubins_path.origin_lat,
						 dubins_path.origin_lon,
						 dubins_path.path_type);
			break;

			/* ACTUATORS */
		case ACTUATORS_VALUES:
		case ACTUATORS_CALIBRATION:
		case ACTUATORS_ROTOR_PARAMS:
		case ACTUATORS_MIXING_PARAMS:

			/* INPUT */
		case INPUT_HANDSET_VALUES:
		case INPUT_HANDSET_SETUP:
		case INPUT_JOYSTICK_VALUES:
		case INPUT_JOYSTICK_SETUP:

			/* SYSTEM */
		case SYSTEM_POWER_ON:
		case SYSTEM_INITIALIZE:
		case SYSTEM_HEALTH_AND_STATUS:
		case SYSTEM_HARDWARE_ERROR:
		case SYSTEM_REBOOT:
			break;

			/* TELEMETRY */
		case TELEMETRY_HEARTBEAT:
			if(show_telemetry)
				printf("TELEMETRY_HEARTBEAT\n");
			break;

		case TELEMETRY_POSITION:
			if(show_telemetry) {
				printf("TELEMETRY_POSITION\n");
				printf("\tLatitude:\t%0.02f\n",telemetry_position.latitude);
				printf("\tLongitude:\t%0.02f\n",telemetry_position.longitude);
				printf("\tAltitude:\t%0.02f\n",telemetry_position.altitude);
			}
			memcpy(&telemetry_position,data,sizeof(TelemetryPosition_t));
			break;

		case TELEMETRY_ORIENTATION:
			if(show_telemetry)
				printf("TELEMETRY_ORIENTATION\n");
			break;
		case TELEMETRY_PRESSURE:
			if(show_telemetry)
				printf("TELEMETRY_PRESSURE\n");
			break;
		case TELEMETRY_CONTROL:
			if(show_telemetry)
				printf("TELEMETRY_CONTROL\n");
			memcpy(&telemetry_control,data,sizeof(TelemetryControl_t));
			break;
		case TELEMETRY_SYSTEM:
			if(show_telemetry)
				printf("TELEMETRY_SYSTEM\n");
			memcpy(&telemetry_system,data,sizeof(TelemetrySystem_t));
			break;
		case TELEMETRY_GCS:
			if(show_telemetry)
				printf("TELEMETRY_GCS\n");
			break;
		case TELEMETRY_GCS_LOCATION:
			if(show_telemetry)
				printf("TELEMETRY_GCS_LOCATION\n");
			break;

			/* ERRORS */
		default:
		case INVALID_PACKET:
			break;
	}
}

uint8_t receiveCommand(uint8_t type, void * data, uint16_t size, const void * parameter)
{
	printf("receiveCommand: type=%u\n", type);

	// validate this is a command
	if( size != sizeof(Command_t) ) {
		printf("receiveCommand: invlid data size - size=%u\n", size);
		return false;
	}

	Command_t *command = (Command_t*)data;;

	switch(command->id) {
		case CMD_PAYLOAD_CONTROL:

			switch((uint8_t)command->value) {
				case PAYLOAD_CTRL_OFF:
					printf("CMD:PAYLOAD_CTRL_OFF\n");
					payload_current_state = PAYLOAD_CTRL_OFF;
					return true;

				case PAYLOAD_CTRL_ACTIVE:
					printf("CMD:PAYLOAD_CTRL_ACTIVE\n");
					if( payload_current_state != PAYLOAD_CTRL_READY )
						return false;

					payload_current_state = PAYLOAD_CTRL_ACTIVE;
					return true;

				case PAYLOAD_CTRL_SHUTDOWN:
					printf("CMD:PAYLOAD_CTRL_SHUTDOWN\n");
					payload_current_state = PAYLOAD_CTRL_SHUTDOWN;
					return true;

				default:
					printf("receiveCommand: unexpected payload command\n");
					break;
			}

			break;

		default:
			break;
	}

	// do some with commands
	return false;
}

void receiveReply(uint8_t type, void * data, uint16_t size, bool ack, const void * parameter)
{
	printf("receiveReply: type=%u\n", type);
	ack ? fprintf(stderr,"--> ACK\n") : fprintf(stderr,"--> NACK\n");

	Command_t * tmp_command = (Command_t *) data;

	if(set_command.id == tmp_command->id) {
		//if (set_command.value == tmp_command->value) {
			set_command_ack = ack;
		//}
		received_reply = true;
	}

	switch(type) {
		case CONTROL_COMMAND:

			switch(tmp_command->id) {

				case CMD_PAYLOAD_CONTROL:

					switch((uint8_t)tmp_command->value) {
						// autopilot ack/nack payload sending off command
						case PAYLOAD_CTRL_OFF:
							printf("PAYLOAD_CTRL_OFF\n");
							break;

							// autopilot ack/nack payload sending ready command
						case PAYLOAD_CTRL_READY:
							printf("PAYLOAD_CTRL_READY\n");
							break;

							// autopilot ack/nack payload sending shutdown command
						case PAYLOAD_CTRL_SHUTDOWN:
							printf("PAYLOAD_CTRL_SHUTDOWN");
							break;

							// autopilot ack/nack payload sending error command
						case PAYLOAD_CTRL_ERROR:
							printf("PAYLOAD_CTRL_ERROR");
							break;

						default:
							//printf("receiveReply: unexpected ack/nack\n");
							break;
					}

					if( ack ) {
						payload_current_state = (PayloadControl_t) tmp_command->value;
						printf(" successful\n");
					} else {
						printf(" failed (CMD_PAYLOAD_CONTROL)\n");
					}

					break;

                case CMD_ALT_MODE:
                    printf("Setting alt mode");
                    if (ack) {
                        printf( " successful\n");
                    } else {
                        printf(" failed (CMD_ALT_MODE)\n");
                    }
                    break;
				case CMD_X_VEL:
					printf("Setting X velocity to %f", tmp_command->value);
					if(ack) printf(" successful\n");
					else printf(" failed (CMD_X_VEL)\n");
					break;
				case CMD_Y_VEL:
					printf("Setting Y velocity to %f", tmp_command->value);
					if(ack) printf(" successful\n");
					else printf(" failed (CMD_Y_VEL)\n");
					break;
				case CMD_VRATE:
					printf("Setting Z velocity to %f", tmp_command->value);
					if(ack) printf(" successful\n");
					else printf(" failed (CMD_VRATE)\n");
					break;

			}
	}
}

void request(uint8_t type, uint8_t value)
{
	printf("request: type=%u\n", type);
	// do some with status request
}

bool publish(uint8_t type, uint8_t param)
{
	//printf("publish: type=%u\n", type);

	// do some with status request
	switch(type) {
		case SYSTEM_INITIALIZE:
			printf("publish: SYSTEM_INITIALIZE\n");

			comm_handler->send(SYSTEM_INITIALIZE, (uint8_t *)&system_init, sizeof(SystemInitialize_t), NULL);
			break;

		default:
			return false;
	}

	return true;
}
