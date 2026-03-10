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

#include "test_handler.h"

#include "test.h"
#include "main.h"
#include "structs.h"

#include "helper_functions.h"
#include "debug.h"

SystemInitialize_t system_init;

// Commands
Command_t set_command;
bool set_command_ack;
bool received_reply = false;

extern bool new_telemetry_system;

TelemetryOrientation_t telemetry_orientation;
TelemetryPosition_t    telemetry_position;
TelemetryPressure_t    telemetry_pressure;
TelemetrySystem_t      telemetry_system;
TelemetryControl_t     telemetry_control;
State_t estimator_data;
SingleValueSensor_t agl_data;
DubinsPath_t dubins_path;

UserPayload_t          rx_payload;

TelemetryPosition_t    telemetry_gcs;

DeploymentTube_t deployment_tube;

S0Sensors_t s0_sensors;

// Calibration
volatile SensorType_t calibration_requested = UNKNOWN_SENSOR;
volatile bool waiting_on_calibrate = false;

extern bool show_telemetry;


void receive(uint8_t type, void * data, uint16_t size, const void * parameter) 
{
	//printf("receive: type=%u\n", type);
	
	float x,y, distance, bearing;
	uint32_t address = ((BSTProtocol *)comm_handler)->getLastAddress();

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
		case SENSORS_CALIBRATE:
		case SENSORS_BOARD_ORIENTATION:
		case SENSORS_GNSS_ORIENTATION:
		case SENSORS_MHP:

			/* STATE */
		case STATE_STATE:
		case STATE_ESTIMATOR_PARAM:

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

			/* HANDSET */
		case HANDSET_VALUES:
		case HANDSET_CALIBRATION:

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
			break;

		case TELEMETRY_POSITION:
			if((address & 0xFF000000) == 0x41000000) {
				memcpy(&telemetry_position,data,sizeof(TelemetryPosition_t));
			}
			if((address & 0xFF000000) == 0x53000000) {
				memcpy(&telemetry_gcs,data,sizeof(TelemetryPosition_t));
			}
			break;

		case TELEMETRY_ORIENTATION:
			memcpy(&telemetry_orientation,data,sizeof(TelemetryOrientation_t));
			break;
		case TELEMETRY_PRESSURE:
			memcpy(&telemetry_pressure,data,sizeof(TelemetryPressure_t));
			break;
		case TELEMETRY_CONTROL:
			memcpy(&telemetry_control,data,sizeof(TelemetryControl_t));
			break;
		case TELEMETRY_SYSTEM:
			new_telemetry_system = true;
			memcpy(&telemetry_system,data,sizeof(TelemetrySystem_t));
			break;
		case TELEMETRY_GCS:
			break;
		case TELEMETRY_GCS_LOCATION:
			break;

		case TELEMETRY_PAYLOAD:
			break;

		case TELEMETRY_DEPLOYMENT_TUBE:
			memcpy(&deployment_tube,data,sizeof(DeploymentTube_t));
			break;

			/* FLIGHT PLAN */
		case FLIGHT_PLAN:
		case FLIGHT_PLAN_MAP:
		case FLIGHT_PLAN_WAYPOINT:
		case LAST_MAPPING_WAYPOINT:
		case DUBIN_PATH:

			/* VEHICLE CONFIGURATION */
		case VEHICLE_PARAMS:
		case VEHICLE_LIMITS:
		case VEHICLE_LAUNCH_PARAMS:
		case VEHICLE_LAND_PARAMS:

			/* MISSION */
		case MISSION_CHECKLIST:
		case MISSION_PARAMETERS:

			/* PAYLOAD */
		case PAYLOAD_TRIGGER:
		case PAYLOAD_PARAMS:
		case PAYLOAD_NDVI:
		case PAYLOAD_LDCR:
		case PAYLOAD_CONTROL:
		case PAYLOAD_CAMERA_TAG:
		case PAYLOAD_STATUS:
			break;

		case PAYLOAD_DATA_CHANNEL_0:
			break;
		case PAYLOAD_DATA_CHANNEL_1:
			memcpy(&rx_payload,data,sizeof(UserPayload_t));
			char out[100];
			break;

		case PAYLOAD_S0_SENSORS:
			memcpy(&s0_sensors,data,sizeof(S0Sensors_t));
			x = LON_TO_M(telemetry_position.longitude - telemetry_gcs.longitude, telemetry_position.latitude); 
			y = LAT_TO_M(telemetry_position.latitude - telemetry_gcs.latitude);

			distance = sqrt(x*x+y*y);

			bearing = angle2heading(270-atan2(y,x)) * 180.0 / M_PI;

			if(show_telemetry) {
			//printf("%05.2f: [%0.1f %0.1f] [%+4.1f %+4.1f %+4.1f %+4.1f %+4.1f] %0.1f %3.1f %5.2f %0.1f  (%0.1f %0.1f)\n",
			printf("%07.2f: [%0.1f %0.1f] [%+4.1f %+4.1f %+4.1f %+4.1f %+4.1f] (%0.1f %4.1f) %5.2f %0.1f <%+06.1f %+06.1f %+06.1f.>\n",
					s0_sensors.system_time / 1000.f,
					s0_sensors.static_pressure[0] / 10.f,
					s0_sensors.static_pressure[1] / 10.f,
					s0_sensors.dynamic_pressure[0] / 10.f,
					s0_sensors.dynamic_pressure[1] / 10.f,
					s0_sensors.dynamic_pressure[2] / 10.f,
					s0_sensors.dynamic_pressure[3] / 10.f,
					s0_sensors.dynamic_pressure[4] / 10.f,
					s0_sensors.air_temperature / 100.f,
					s0_sensors.humidity / 100.f,
					s0_sensors.laser_distance / 100.f,
					s0_sensors.ground_temperature / 100.f,
					//distance, bearing);
					s0_sensors.u / 100.f,
					s0_sensors.v / 100.f,
					s0_sensors.w / 100.f);
			}

			break;


			/* ERRORS */
		default:
		case INVALID_PACKET:
			break;
	}
}

uint8_t receiveCommand(uint8_t type, void * data, uint16_t size, const void * parameter)
{

	// validate this is a command
	if( size != sizeof(Command_t) ) {
		pmesg(VERBOSE_ERROR, "receiveCommand: invalid data size - size=%u\n", size);
		return false;
	}

	Command_t *command = (Command_t*)data;;

	switch(command->id) {
		default:
			printf("receiveCommand: type=%u\n", type);
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

	if(set_command.id == tmp_command->id || (type == set_command.id) ) {
		//if (set_command.value == tmp_command->value) {
			set_command_ack = ack;
		//}
		received_reply = true;
	}

	switch(type) {
		case CONTROL_COMMAND:

			switch(tmp_command->id) {

				case CMD_ALT_MODE:
					printf("Setting alt mode");
					if (ack) printf( " successful\n");
					else printf(" failed (CMD_ALT_MODE)\n");
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
			printf("publish: SYSTEM_INITIALIZE\n");

			comm_handler->send(SYSTEM_INITIALIZE, (uint8_t *)&system_init, sizeof(SystemInitialize_t), NULL);
			break;

		default:
			return false;
	}

	return true;
}
//---- Heartbeat Interface
static float last_heartbeat = 0.0;

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
bool setCommandValue(bst::comms::CommandID_t id, float value, bool blocking) {
    set_command.id = id;
    set_command.value = value;

		set_command_ack = false;
		received_reply = false;

    comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&set_command, sizeof(Command_t), NULL);

		if(!blocking) return true;

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

//---- Validate the navigation mode is in a PILOT mode for SDK control
bool validate_nav_mode() 
{
    if (telemetry_control.nav_mode != NAV_PILOT_BODY && telemetry_control.nav_mode != NAV_PILOT_WORLD) {
        printf("YOU NEED TO CHANGE TO VELOCITY CONTROL MODE ('V' command): mode=%u\n", telemetry_control.nav_mode);
        return false;
    }

    return true;
}

//---- Validate the altitude mode is in RATE mode for SDK control
bool validate_alt_rate_mode() 
{
    if (telemetry_control.alt_mode != ALT_MODE_RATE ) {
        printf("YOU NEED TO CHANGE TO ALTITUDE RATE CONTROL MODE ('A' command): mode=%u\n", telemetry_control.alt_mode);
        return false;
    }

    return true;
}
