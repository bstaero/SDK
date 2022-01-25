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
#include <unistd.h>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <vector>

#include "debug.h"
#include "test.h"
#include "main.h"
#include "structs.h"
#include "test_handler.h"

SystemInitialize_t system_init;

bool show_telemetry = false;

// Payload state
PayloadControl_t payload_current_request = PAYLOAD_CTRL_OFF;
PayloadControl_t payload_current_state = PAYLOAD_CTRL_OFF;

// Commands
Command_t set_command;
bool set_command_ack;
bool received_reply = false;

// System state
TelemetryPosition_t telemetry_position;
TelemetrySystem_t   telemetry_system;
TelemetryControl_t  telemetry_control;
CalibrateSensor_t * calibration_data;
State_t estimator_data;
SingleValueSensor_t agl_data;
DubinsPath_t dubins_path;

// Calibration
volatile SensorType_t calibration_requested = UNKNOWN_SENSOR;
volatile bool waiting_on_calibrate = false;

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
				printf("\tLaser:\t:%0.02f\n",agl_data.value);
			break;

		case SENSORS_CALIBRATE:
			//printf(" Sensor calibration ...");
			calibration_data = (CalibrateSensor_t *)data;
			if(calibration_data->state == CALIBRATED) {
				if(calibration_data->sensor == calibration_requested) {
					printf(" calibrated\n");
					calibration_requested = UNKNOWN_SENSOR;
				}
			} else
				printf(" failed\n");
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
				printf("[%.2f] STATE_ESTIMATOR_PARAM\n", getElapsedTime() );
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
				printf("[%.2f] TELEMETRY_HEARTBEAT\n", getElapsedTime() );
			break;

		case TELEMETRY_POSITION:
			if(show_telemetry) {
				printf("[%.2f] TELEMETRY_POSITION\n", getElapsedTime() );
				printf("\tLatitude:\t%0.02f\n",telemetry_position.latitude);
				printf("\tLongitude:\t%0.02f\n",telemetry_position.longitude);
				printf("\tAltitude:\t%0.02f\n",telemetry_position.altitude);
			}
			memcpy(&telemetry_position,data,sizeof(TelemetryPosition_t));
			break;

		case TELEMETRY_ORIENTATION:
			if(show_telemetry)
				printf("[%.2f] TELEMETRY_ORIENTATION\n", getElapsedTime() );
			break;
		case TELEMETRY_PRESSURE:
			if(show_telemetry)
				printf("[%.2f] TELEMETRY_PRESSURE\n", getElapsedTime() );
			break;
		case TELEMETRY_CONTROL:
			if(show_telemetry)
				printf("[%.2f] TELEMETRY_CONTROL\n", getElapsedTime() );
			memcpy(&telemetry_control,data,sizeof(TelemetryControl_t));
			break;
		case TELEMETRY_SYSTEM:
			if(show_telemetry)
				printf("[%.2f] TELEMETRY_SYSTEM\n", getElapsedTime() );
			memcpy(&telemetry_system,data,sizeof(TelemetrySystem_t));
			break;
		case TELEMETRY_GCS:
			if(show_telemetry)
				printf("[%.2f] TELEMETRY_GCS\n", getElapsedTime() );
			break;
		case TELEMETRY_GCS_LOCATION:
			if(show_telemetry)
				printf("[%.2f] TELEMETRY_GCS_LOCATION\n", getElapsedTime() );
			break;

			/* ERRORS */
		default:
		case INVALID_PACKET:
			break;
	}
}

uint8_t receiveCommand(uint8_t type, void * data, uint16_t size, const void * parameter)
{
	//printf("receiveCommand: type=%u\n", type);

	// validate this is a command
	if( size != sizeof(Command_t) ) {
		pmesg(VERBOSE_ERROR, "receiveCommand: invlid data size - size=%u\n", size);
		return false;
	}

	Command_t *command = (Command_t*)data;;

	switch(command->id) {
		case CMD_PAYLOAD_CONTROL:

			switch((uint8_t)command->value) {
				case PAYLOAD_CTRL_OFF:
					pmesg(VERBOSE_PACKETS, "CMD:PAYLOAD_CTRL_OFF\n");
					payload_current_state = PAYLOAD_CTRL_OFF;
					return true;

				case PAYLOAD_CTRL_ACTIVE:
					pmesg(VERBOSE_PACKETS, "CMD:PAYLOAD_CTRL_ACTIVE\n");
					if( payload_current_state != PAYLOAD_CTRL_READY )
						return false;

					payload_current_state = PAYLOAD_CTRL_ACTIVE;
					return true;

				case PAYLOAD_CTRL_SHUTDOWN:
					pmesg(VERBOSE_PACKETS,"CMD:PAYLOAD_CTRL_SHUTDOWN\n");
					payload_current_state = PAYLOAD_CTRL_SHUTDOWN;
					return true;

				default:
					pmesg(VERBOSE_WARN,"receiveCommand: unexpected payload command\n");
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

	Command_t * tmp_command = (Command_t *) data;

	//printf("receiveReply: type=%u\n", type);
	//ack ? fprintf(stderr,"--> ACK [%i, %i]\n", set_command.id, tmp_command->id) : fprintf(stderr,"--> NACK[%i, %i]\n", set_command.id, tmp_command->id);

	if(set_command.id == tmp_command->id || (tmp_command->id == 255 && type == set_command.id) ) {
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
							pmesg(VERBOSE_PACKETS, "PAYLOAD_CTRL_OFF");
							break;

							// autopilot ack/nack payload sending ready command
						case PAYLOAD_CTRL_READY:
							pmesg(VERBOSE_PACKETS, "PAYLOAD_CTRL_READY");
							break;

							// autopilot ack/nack payload sending shutdown command
						case PAYLOAD_CTRL_SHUTDOWN:
							pmesg(VERBOSE_PACKETS, "PAYLOAD_CTRL_SHUTDOWN");
							break;

							// autopilot ack/nack payload sending error command
						case PAYLOAD_CTRL_ERROR:
							pmesg(VERBOSE_PACKETS, "PAYLOAD_CTRL_ERROR");
							break;

						default:
							//printf("receiveReply: unexpected ack/nack\n");
							break;
					}

					if( ack ) {
						payload_current_state = (PayloadControl_t) tmp_command->value;
						if( verbose >= VERBOSE_PACKETS )
							printf(" successful\n");
					} else {
						if( verbose >= VERBOSE_ERROR )
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
				case CMD_YAW:
					printf("Setting heading to %0.01f", tmp_command->value * 180.0/M_PI);
					if(ack) printf(" successful\n");
					else printf(" failed (CMD_YAW)\n");
					break;


			}
			break;

		case SENSORS_CALIBRATE:
			set_command_ack = ack;
			received_reply = true;
			waiting_on_calibrate = false;

			// printf("Sensors calibrate [%u] ", tmp_command->id);
			// if(ack) printf(" successful\n");
			// else printf(" failed\n");
			break;
	}
}

void request(uint8_t type, uint8_t value)
{
	//printf("request: type=%u\n", type);
	// do some with status request
}

bool publish(uint8_t type, uint8_t param)
{
	//printf("publish: type=%u\n", type);

	// do some with status request
	switch(type) {
		case SYSTEM_INITIALIZE:
			//printf("publish: SYSTEM_INITIALIZE\n");

			comm_handler->send(SYSTEM_INITIALIZE, (uint8_t *)&system_init, sizeof(SystemInitialize_t), NULL);
			break;

		default:
			return false;
	}

	return true;
}
//---- Heartbeat Interface
static float last_heartbeat = 0.0;

void send_hb()
{
    // send heartbeat
    float now = getElapsedTime();

    /** Update heartbeat */
    if (now - last_heartbeat > HEARTBEAT_INTERVAL) {
        last_heartbeat = now;
        comm_handler->send(TELEMETRY_HEARTBEAT, NULL, 0, NULL);
    }
}

//---- Wait for message ACK 
// Must set these two variables before calling
//    set_command_ack = false;
//    received_reply = false;

bool waitForACK() 
{
    float sent_time = getElapsedTime();
    while (!received_reply && getElapsedTime() - sent_time < CMD_ACK_TIMEOUT) {
        usleep(1000);
        comm_handler->update();
        send_hb();
    }

    if (!received_reply) {
        pmesg(VERBOSE_WARN, " Failed: no response!\n");
        return false;
    }

    received_reply = false;

    if (!set_command_ack) {
        pmesg(VERBOSE_ERROR, " Failed: got NACK!\n");
        return false;
    }

    return true;
}


//---- Send command and wait for ACK
bool setCommandValue(bst::comms::CommandID_t id, float value) {
    set_command.id = id;
    set_command.value = value;

    set_command_ack = false;
    received_reply = false;

    comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&set_command, sizeof(Command_t), NULL);

    return waitForACK();
}

//---- Send command and wait for valid ACK and a value to change
// This function waits for both the ACK from the command, and for a specific
// value to change. This function is setup for validating uint8_t types at this time.
// The timeout value is how long to wait for the value to change. It does not control the 
// timeout waiting for the ACK 
bool setCheckCommandValue(uint8_t id, int value, uint8_t* check, uint8_t check_value, float timeout ) {

    if (!setCommandValue((bst::comms::CommandID_t) id, value)) {
        pmesg(VERBOSE_ERROR, "command failed\n");
        return false;
    }

    float sent_time = getElapsedTime();
    while (*check != check_value && getElapsedTime() - sent_time < timeout) {
        usleep(1000);
        comm_handler->update();
        send_hb();
    }
    if (*check != check_value) {
        pmesg(VERBOSE_ERROR, "failed value validation\n");
        return false;
    }
    return true;

}

//---- Send flight plan and wait for ACK
bool sendFlightPlan(uint8_t *temp_waypoints, uint8_t num_points, FlightPlanMap_t *flight_plan_map)
{
    set_command_ack = false;
    received_reply = false;
    set_command.id = FLIGHT_PLAN;

    comm_handler->sendCommand(FLIGHT_PLAN, (uint8_t *)temp_waypoints, num_points, flight_plan_map);

    return waitForACK();
}

bool sendPayloadControlMode(PayloadControl_t control_value) {
    Command_t command;

    // maintain our requested state
    payload_current_request = control_value;

    return setCommandValue((bst::comms::CommandID_t) CMD_PAYLOAD_CONTROL, control_value);

    // command.id = CMD_PAYLOAD_CONTROL;
    // command.value = control_value;
    // comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);

    // return true;
}

bool sendCalibrate(SensorType_t sensor) {
    if (calibration_requested != UNKNOWN_SENSOR) {
        pmesg(VERBOSE_ERROR, "Failed: pending request\n");
        return false;
    }

    CalibrateSensor_t calibrate_pkt;

    switch (sensor) {
        case GYROSCOPE:
            break;
        default:
			pmesg(VERBOSE_ERROR, "Failed: Can only currently zero gyroscopes!\n");
            return false;
    }

    calibration_requested = sensor;
    calibrate_pkt.sensor = sensor;
    calibrate_pkt.state = SENT;

    set_command_ack = false;
    received_reply = false;
    waiting_on_calibrate = true;
    set_command.id = SENSORS_CALIBRATE;

    //printf("sending calibration packet\n");
    comm_handler->sendCommand(SENSORS_CALIBRATE, (uint8_t *)&calibrate_pkt, sizeof(CalibrateSensor_t), NULL);

    return waitForACK();
}

//---- Validate the payload state is in ACTIVE state
// Active state is required to control the vehicle from the SDK
bool validate_payload_control() 
{
    // validate payload interface is in good state
    if ( payload_current_state != PAYLOAD_CTRL_ACTIVE && payload_current_state != PAYLOAD_CTRL_READY) {
        pmesg(VERBOSE_WARN, "PAYLOAD IS NOT IN ACTIVE OR READY MODE!\n");
        return false;
    }

    // Go to active payload control if needed
    if (payload_current_state != PAYLOAD_CTRL_ACTIVE) {
        pmesg(VERBOSE_PAYLOAD, "Activating payload control ...");

        if (!setCheckCommandValue(CMD_PAYLOAD_CONTROL, PAYLOAD_CTRL_ACTIVE, (uint8_t*)&payload_current_state, PAYLOAD_CTRL_ACTIVE, 10.0)) {
			if( verbose >= VERBOSE_PAYLOAD)
				printf(" failed\n");
            return false;
        }
		else if( verbose >= VERBOSE_PAYLOAD)
			printf(" done\n");
    }

    return true;
}

//---- Validate the navigation mode is in a PILOT mode for SDK control
bool validate_nav_mode() 
{
    // validate payload interface is in good state
    if( !validate_payload_control() )
        return false;

    if (telemetry_control.nav_mode != NAV_PILOT_BODY && telemetry_control.nav_mode != NAV_PILOT_WORLD) {
        printf("YOU NEED TO CHANGE TO VELOCITY CONTROL MODE ('v' command): mode=%u\n", telemetry_control.nav_mode);
        return false;
    }

    return true;
}

//---- Validate the altitude mode is in RATE mode for SDK control
bool validate_alt_rate_mode() 
{
    // validate payload interface is in good state
    if( !validate_payload_control() )
        return false;

    if (telemetry_control.alt_mode != ALT_MODE_RATE ) {
        printf("YOU NEED TO CHANGE TO ALTITUDE RATE CONTROL MODE ('A' command): mode=%u\n", telemetry_control.alt_mode);
        return false;
    }

    return true;
}
