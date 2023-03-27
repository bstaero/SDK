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

#include "test.h"
#include "test_handler.h"
#include "main.h"
#include "structs.h"

TelemetryOrientation_t telemetry_orientation;
TelemetryPosition_t    telemetry_position;
TelemetryPressure_t    telemetry_pressure;
TelemetrySystem_t      telemetry_system;
TelemetryControl_t     telemetry_control;

UserPayload_t          rx_payload;

bool new_data = false;
Downstream_t downstream_pkt;

void receive(uint8_t type, void * data, uint16_t size, const void * parameter) 
{
	//printf("receive: type=%u\n", type);

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
			memcpy(&telemetry_position,data,sizeof(TelemetryPosition_t));
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
			memcpy(&telemetry_system,data,sizeof(TelemetrySystem_t));
			break;
		case TELEMETRY_GCS:
			break;
		case TELEMETRY_GCS_LOCATION:
			break;

		case TELEMETRY_PAYLOAD:
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
			memcpy(&rx_payload,data,sizeof(UserPayload_t));
			char out[100];

			//snprintf(out,rx_payload.size+1,
					//"%s",(char*)rx_payload.buffer);
			//printf("Got %i bytes from the payload: [%s]\n",rx_payload.size,out);
			//printf("%s",out);

			memcpy(&downstream_pkt,rx_payload.buffer,sizeof(Downstream_t));
			new_data = true;

			break;

		case PAYLOAD_DATA_CHANNEL_1:
		case PAYLOAD_DATA_CHANNEL_2:
		case PAYLOAD_DATA_CHANNEL_3:
		case PAYLOAD_DATA_CHANNEL_4:
		case PAYLOAD_DATA_CHANNEL_5:
		case PAYLOAD_DATA_CHANNEL_6:
		case PAYLOAD_DATA_CHANNEL_7:

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

	// do something with commands
	return false;
}

void receiveReply(uint8_t type, void * data, uint16_t size, bool ack, const void * parameter) 
{
	printf("receiveReply: type=%u\n", type);
	ack? fprintf(stderr,"ACK\n"): fprintf(stderr,"NACK\n");

	Command_t * tmp_command = (Command_t *) data;
}

void request(uint8_t type, uint8_t value) 
{
	printf("request: type=%u\n", type);
	// do something with status request
}

bool publish(uint8_t type, uint8_t param) 
{
	printf("publish: type=%u\n", type);

	// do something with status request
	return true;
}
