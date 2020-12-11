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

extern PayloadControl_t    payload_current_state;

TelemetryPosition_t telemetry_position;
TelemetrySystem_t   telemetry_system;
TelemetryControl_t  telemetry_control;

CalibrateSensor_t * calibration_data;

State_t * estimator_data;
SingleValueSensor_t * agl_data;

/** Control parameters */
float agl_set_point = 15.0;
float Pgain = 0.125;
float Igain = 0.0;
float Dgain = 0.55;
float Sgain = 0.0;
float agl_error = 0.0;
float agl_error_z1 = 0.0;
float agl_diff;
float agl_int;
float agl_windup_limit = 100.0;
float laser_pitch_offset = 0.0*M_PI/180.0; //radians
float dt = 0.1; // s, update rate for agl laser estimate
float vel_command;
float q0 = 1.0;
float q1 = 0.0;
float q2 = 0.0;
float q3 = 0.0;
float max_vel = 3.0;
float min_vel = -1.0;
int debug_count = 0;
int debug_count_period = 100;
std::vector<float> agl_history(10, 0.0);
std::vector<float> time_history(10, 0.0);
unsigned long agl_history_count = 0;

float slope(const std::vector<float>& x, const std::vector<float>& y) {
    const auto n    = x.size();
    const auto s_x  = std::accumulate(x.begin(), x.end(), 0.0);
    const auto s_y  = std::accumulate(y.begin(), y.end(), 0.0);
    const auto s_xx = std::inner_product(x.begin(), x.end(), x.begin(), 0.0);
    const auto s_xy = std::inner_product(x.begin(), x.end(), y.begin(), 0.0);
    const auto a    = (n * s_xy - s_x * s_y) / (n * s_xx - s_x * s_x);
    return a;
}


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
            break;

		case SENSORS_CALIBRATE:
			calibration_data = (CalibrateSensor_t *)data;
			if(calibration_data->state == CALIBRATED)
				if(calibration_data->sensor == calibration_requested)
					calibration_requested = UNKNOWN_SENSOR;
			break;

#ifdef VEHICLE_FIXEDWING
		case SENSORS_OFFSETS:
		case SENSORS_ORIENTATION:
#endif

			/* STATE */
		case STATE_STATE:
		case STATE_ESTIMATOR_PARAM:
            estimator_data = (State_t *) data;
            q0 = estimator_data->q[0];
            q1 = estimator_data->q[1];
            q2 = estimator_data->q[2];
            q3 = estimator_data->q[3];
            break;

			/* CONTROL */
		case CONTROL_COMMAND:
		case CONTROL_PID:
		case CONTROL_FLIGHT_PARAMS:
		case CONTROL_FILTER_PARAMS:

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
			//printf("TELEMETRY_HEARTBEAT\n");
			break;

		case TELEMETRY_POSITION:
			//printf("TELEMETRY_POSITION\n");
			memcpy(&telemetry_position,data,sizeof(TelemetryPosition_t));
			/*printf("\tLatitude:\t%0.02f\n",telemetry_position.latitude);
			printf("\tLongitude:\t%0.02f\n",telemetry_position.longitude);
			printf("\tAltitude:\t%0.02f\n",telemetry_position.altitude);*/
			break;

		case TELEMETRY_ORIENTATION:
			//printf("TELEMETRY_ORIENTATION\n");
			break;
		case TELEMETRY_PRESSURE:
			//printf("TELEMETRY_PRESSURE\n");
			break;
		case TELEMETRY_CONTROL:
			//printf("TELEMETRY_CONTROL\n");
			memcpy(&telemetry_control,data,sizeof(TelemetryControl_t));
			break;
		case TELEMETRY_SYSTEM:
			//printf("TELEMETRY_SYSTEM\n");
			memcpy(&telemetry_system,data,sizeof(TelemetrySystem_t));
			break;
		case TELEMETRY_GCS:
			//printf("TELEMETRY_GCS\n");
			break;
		case TELEMETRY_GCS_LOCATION:
			//printf("TELEMETRY_GCS_LOCATION\n");
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
					//printf("Setting Z velocity to %f", tmp_command->value);
					//if(ack) printf(" successful\n");
					//else printf(" failed (CMD_VRATE)\n");
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
	printf("publish: type=%u\n", type);

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
