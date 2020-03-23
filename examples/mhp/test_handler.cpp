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
#include <string.h>

#include "test_handler.h"

#include "test.h"
#include "main.h"
#include "structs.h"

/*<---Global Variables---->*/
Packet rx_packet;
Packet tx_packet;

volatile SensorType_t calibration_requested = UNKNOWN_SENSOR;
/*<-End Global Variables-->*/

/*<---Local Functions----->*/
void handlePacket(uint8_t type, const void * data);
/*<-End Local Functions--->*/

void updateCommunications(void) {

	uint8_t data;
	rx_packet.setAddressing(false);

	while(readByte(&data)) {
		if(rx_packet.isValid(data))
			handlePacket(rx_packet.getType(), rx_packet.getDataPtr());
	}
}

void handlePacket(uint8_t type, const void * data) 
{
	//printf("handlePacket: type=%u\n", type);
	MHP_Time_t * mhp_data;
	PowerOn_t * power_on_data;
	CalibrateSensor_t * calibration_data;

	switch(type) {

		case SENSORS_CALIBRATE:
			calibration_data = (CalibrateSensor_t *)data;
			if(calibration_data->state == CALIBRATED)
				if(calibration_data->sensor == calibration_requested)
					calibration_requested = UNKNOWN_SENSOR;
			break;

		case SENSORS_MHP:
			mhp_data = (MHP_Time_t *)data;

			if(display_telemetry)
				printf("%+0.01f m %+05.01f %+05.01f %+05.01f a %+0.02f %+0.02f %+0.02f g %+0.02f %+0.02f %+0.02f d %+07.01f %+07.01f %+07.01f %+07.01f %+07.01f %+0.01f %0.01f a %+05.01f b %+05.01f q %+07.01f i %+05.01f t %+05.01f\n\r",
						mhp_data->static_pressure,
						mhp_data->magnetometer[0],
						mhp_data->magnetometer[1],
						mhp_data->magnetometer[2],
						mhp_data->accelerometer[0],
						mhp_data->accelerometer[1],
						mhp_data->accelerometer[2],
						mhp_data->gyroscope[0],
						mhp_data->gyroscope[1],
						mhp_data->gyroscope[2],
						mhp_data->dynamic_pressure[0],
						mhp_data->dynamic_pressure[1],
						mhp_data->dynamic_pressure[2],
						mhp_data->dynamic_pressure[3],
						mhp_data->dynamic_pressure[4],
						mhp_data->air_temperature,
						mhp_data->humidity,
						mhp_data->alpha*180.0/M_PI,
						mhp_data->beta*180.0/M_PI,
						mhp_data->q,
						mhp_data->ias,
						mhp_data->tas
						);

			if(display_telemetry_timing)
				printf("s %+0.02f d %+0.02f %+0.02f %+0.02f %+0.02f %+0.02f t %+0.02f h %+0.02f i %+0.02f m %+0.02f \n\r",
						mhp_data->static_pressure_time,  // [s]
						mhp_data->dynamic_pressure_time[0],  // [s]
						mhp_data->dynamic_pressure_time[1],  // [s]
						mhp_data->dynamic_pressure_time[2],  // [s]
						mhp_data->dynamic_pressure_time[3],  // [s]
						mhp_data->dynamic_pressure_time[4],  // [s]
						mhp_data->air_temperature_time,  // [s]
						mhp_data->humidity_time,  // [s]
						mhp_data->imu_time, // [s]
						mhp_data->magnetometer_time // [s]
						);
			break;

		case SYSTEM_POWER_ON:
			power_on_data = (PowerOn_t *)data;

				printf("serial number: 0x%x  comm revision: %u\n",
						power_on_data->serial_num,
						power_on_data->comms_rev
							);
			break;

			/* ERRORS */
		default:
		case INVALID_PACKET:
			break;
	}
}

void sendCalibrate(SensorType_t sensor) {
	if(calibration_requested != UNKNOWN_SENSOR) {
		return;
	}

	char sensor_name[32];
	CalibrateSensor_t calibrate_pkt;

	switch(sensor) {
		case GYROSCOPE:
			break;
		case DYNAMIC_PRESSURE:
			break;
		default:
			return;
	}

	calibration_requested = sensor;

	calibrate_pkt.sensor = sensor;
	calibrate_pkt.state = REQUESTED;

	tx_packet.setAddressing(false);
	tx_packet.setType(SENSORS_CALIBRATE);
	tx_packet.setData((uint8_t *)&calibrate_pkt, sizeof(CalibrateSensor_t));

	writeBytes(tx_packet.getPacket(), tx_packet.getSize());
}

void requestPowerOn(void) {
	PowerOn_t power_on_pkt;

	tx_packet.setAddressing(false);
	tx_packet.setType(SYSTEM_POWER_ON);
	tx_packet.setData((uint8_t *)&power_on_pkt, sizeof(PowerOn_t));

	writeBytes(tx_packet.getPacket(), tx_packet.getSize());
}
