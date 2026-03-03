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

TelemetryOrientation_t telemetry_orientation;
TelemetryPosition_t    telemetry_position;
TelemetryPressure_t    telemetry_pressure;
TelemetrySystem_t      telemetry_system;
TelemetryControl_t     telemetry_control;

UserPayload_t          rx_payload;

void receive(uint8_t type, void * data, uint16_t size, const void * parameter)
{
	//printf("receive: type=%u\n", type);

	switch(type) {
		/* SENSORS */
		case SENSORS_GPS:                // GPS_t
		case SENSORS_ACCELEROMETER:      // ThreeAxisSensor_t
		case SENSORS_GYROSCOPE:          // ThreeAxisSensor_t
		case SENSORS_MAGNETOMETER:       // ThreeAxisSensor_t
		case SENSORS_IMU:                // IMU_t
		case SENSORS_DYNAMIC_PRESSURE:   // Pressure_t
		case SENSORS_STATIC_PRESSURE:    // Pressure_t
		case SENSORS_AIR_TEMPERATURE:    // float
		case SENSORS_AGL:                // SingleValueSensor_t
		case SENSORS_CALIBRATE:          // CalibrateSensor_t
		case SENSORS_BOARD_ORIENTATION:  // ThreeAxisSensor_t
		case SENSORS_GNSS_ORIENTATION:   // ThreeAxisSensor_t
		case SENSORS_MHP:                // MHP_t

			/* STATE */
		case STATE_STATE:                // State_t
		case STATE_ESTIMATOR_PARAM:      // EstimatorParam_t

			/* CONTROL */
		case CONTROL_COMMAND:            // Command_t
		case CONTROL_PID:                // PID_t
		case CONTROL_FLIGHT_PARAMS:      // FlightControlParameters_t
		case CONTROL_FILTER_PARAMS:      // FilterParameters_t

			/* ACTUATORS */
		case ACTUATORS_VALUES:           // ActuatorValues_t
		case ACTUATORS_CALIBRATION:      // ActuatorCalibration_t
		case ACTUATORS_ROTOR_PARAMS:     // RotorParameters_t
		case ACTUATORS_MIXING_PARAMS:    // MixingParameters_t

			/* HANDSET */
		case HANDSET_VALUES:             // HandsetValues_t
		case HANDSET_CALIBRATION:        // HandsetCalibration_t

			/* INPUT */
		case INPUT_HANDSET_VALUES:       // HandsetValues_t
		case INPUT_HANDSET_SETUP:        // HandsetSetup_t
		case INPUT_JOYSTICK_VALUES:      // JoystickValues_t
		case INPUT_JOYSTICK_SETUP:       // JoystickSetup_t

			/* SYSTEM */
		case SYSTEM_POWER_ON:            // SystemPowerOn_t
		case SYSTEM_INITIALIZE:          // SystemInitialize_t
		case SYSTEM_HEALTH_AND_STATUS:   // SystemStatus_t
		case SYSTEM_HARDWARE_ERROR:      // HardwareError_t
		case SYSTEM_REBOOT:              // (empty)
			break;

			/* TELEMETRY */
		case TELEMETRY_HEARTBEAT:
			break;

		case TELEMETRY_POSITION:         // TelemetryPosition_t
			memcpy(&telemetry_position,data,sizeof(TelemetryPosition_t));
			break;

		case TELEMETRY_ORIENTATION:      // TelemetryOrientation_t
			memcpy(&telemetry_orientation,data,sizeof(TelemetryOrientation_t));
			break;
		case TELEMETRY_PRESSURE:         // TelemetryPressure_t
			memcpy(&telemetry_pressure,data,sizeof(TelemetryPressure_t));
			break;
		case TELEMETRY_CONTROL:          // TelemetryControl_t
			memcpy(&telemetry_control,data,sizeof(TelemetryControl_t));
			break;
		case TELEMETRY_SYSTEM:           // TelemetrySystem_t
			memcpy(&telemetry_system,data,sizeof(TelemetrySystem_t));
			break;
		case TELEMETRY_GCS:              // TelemetryGCS_t
			break;
		case TELEMETRY_GCS_LOCATION:     // TelemetryPosition_t
			break;

		case TELEMETRY_PAYLOAD:          // TelemetryPayload_t
			break;

			/* FLIGHT PLAN */
		case FLIGHT_PLAN:                // FlightPlan_t
		case FLIGHT_PLAN_MAP:            // FlightPlanMap_t
		case FLIGHT_PLAN_WAYPOINT:       // Waypoint_t
		case LAST_MAPPING_WAYPOINT:      // uint8_t
		case DUBIN_PATH:                 // DubinsPath_t

			/* VEHICLE CONFIGURATION */
		case VEHICLE_PARAMS:             // VehicleParameters_t
		case VEHICLE_LIMITS:             // VehicleLimits_t
		case VEHICLE_LAUNCH_PARAMS:      // LaunchParameters_t
		case VEHICLE_LAND_PARAMS:        // LandParameters_t

			/* MISSION */
		case MISSION_CHECKLIST:          // MissionChecklist_t
		case MISSION_PARAMETERS:         // MissionParameters_t

			/* PAYLOAD */
		case PAYLOAD_TRIGGER:            // PayloadTrigger_t
		case PAYLOAD_PARAMS:             // PayloadParameters_t
		case PAYLOAD_NDVI:               // PayloadNDVI_t
		case PAYLOAD_LDCR:               // PayloadLDCR_t
		case PAYLOAD_CONTROL:            // PayloadControl_t
		case PAYLOAD_CAMERA_TAG:         // PayloadCameraTag_t
		case PAYLOAD_STATUS:             // PayloadStatus_t
			break;

		case PAYLOAD_DATA_CHANNEL_0:     // UserPayload_t
			memcpy(&rx_payload,data,sizeof(UserPayload_t));
			char out[100];

			snprintf(out,rx_payload.size+1,
					"%s",(char*)rx_payload.buffer);
			printf("Got %i bytes from the payload: [%s]\n",rx_payload.size,out);

			break;

		case PAYLOAD_DATA_CHANNEL_1:     // UserPayload_t
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

	if( size != sizeof(Command_t) ) {
		printf("receiveCommand: invalid data size - size=%u\n", size);
		return false;
	}

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
}

bool publish(uint8_t type, uint8_t param)
{
	printf("publish: type=%u\n", type);

	return true;
}
