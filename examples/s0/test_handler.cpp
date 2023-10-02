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
#include "main.h"
#include "structs.h"

#include "helper_functions.h"

TelemetryOrientation_t telemetry_orientation;
TelemetryPosition_t    telemetry_position;
TelemetryPressure_t    telemetry_pressure;
TelemetrySystem_t      telemetry_system;
TelemetryControl_t     telemetry_control;

UserPayload_t          rx_payload;

TelemetryPosition_t    telemetry_gcs;

S0Sensors_t s0_sensors;

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
		case PAYLOAD_DATA_CHANNEL_1:
		case PAYLOAD_DATA_CHANNEL_2:
		case PAYLOAD_DATA_CHANNEL_3:
		case PAYLOAD_DATA_CHANNEL_4:
		case PAYLOAD_DATA_CHANNEL_5:
		case PAYLOAD_DATA_CHANNEL_6:
		case PAYLOAD_DATA_CHANNEL_7:
			memcpy(&rx_payload,data,sizeof(UserPayload_t));
			char out[100];

			s0_sensors = (S0Sensors_t *)rx_payload.buffer;

			x = LON_TO_M(telemetry_position.longitude - telemetry_gcs.longitude, telemetry_position.latitude); 
			y = LAT_TO_M(telemetry_position.latitude - telemetry_gcs.latitude);

			distance = sqrt(x*x+y*y);

			bearing = angle2heading(270-atan2(y,x)) * 180.0 / M_PI;

			//printf("%05.2f: [%0.1f %0.1f] [%+4.1f %+4.1f %+4.1f %+4.1f %+4.1f] %0.1f %3.1f %5.2f %0.1f  (%0.1f %0.1f)\n",
			printf("%07.2f: [%0.1f %0.1f] [%+4.1f %+4.1f %+4.1f %+4.1f %+4.1f] (%0.1f %4.1f) %5.2f %0.1f (%5.2f %5.2f) <%+06.1f %+06.1f %+06.1f.>\n",
					s0_sensors->system_time,
					s0_sensors->static_pressure[0],
					s0_sensors->static_pressure[1],
					s0_sensors->dynamic_pressure[0] / 10.f,
					s0_sensors->dynamic_pressure[1] / 10.f,
					s0_sensors->dynamic_pressure[2] / 10.f,
					s0_sensors->dynamic_pressure[3] / 10.f,
					s0_sensors->dynamic_pressure[4] / 10.f,
					s0_sensors->air_temperature / 100.f,
					s0_sensors->humidity / 100.f,
					s0_sensors->laser_distance / 100.f,
					s0_sensors->ground_temperature / 100.f,
					s0_sensors->altitude[0] / 10.f,
					s0_sensors->altitude[1] / 10.f,
					//distance, bearing);
					s0_sensors->u / 100.f,
					s0_sensors->v / 100.f,
					s0_sensors->w / 100.f);

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
