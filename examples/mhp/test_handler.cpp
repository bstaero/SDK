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

MHP_t            mhp_data;
MHPSensors_t     mhp_sensors;
MHPSensorsGNSS_t mhp_sensors_gnss;
MHPTiming_t      mhp_timing;

volatile SensorType_t calibration_requested = UNKNOWN_SENSOR;
/*<-End Global Variables-->*/

/*<---Local Functions----->*/
void handlePacket(uint8_t type, const void * data);
/*<-End Local Functions--->*/

bool updateCommunications(void) {

	uint8_t data;
	bool retval = false;
	rx_packet.setAddressing(false);

	while(readByte(&data)) {
		retval = true;
		if(rx_packet.isValid(data)) {
			handlePacket(rx_packet.getType(), rx_packet.getDataPtr());
			break;
		}
	}

	return retval;
}

void handlePacket(uint8_t type, const void * data) 
{
	//printf("handlePacket: type=%u\n", type);

	PowerOn_t * power_on_data;
	CalibrateSensor_t * calibration_data;

	static bool first_run = true;
	char out[1024];

	switch(type) {

		case SENSORS_CALIBRATE:
			calibration_data = (CalibrateSensor_t *)data;
			if(calibration_data->state == CALIBRATED)
				if(calibration_data->sensor == calibration_requested)
					calibration_requested = UNKNOWN_SENSOR;
			break;

		case SENSORS_MHP:
			memcpy(&mhp_data,data,sizeof(MHP_t));;

			if(display_telemetry)
				printf("%+0.01f m %+05.01f %+05.01f %+05.01f a %+0.02f %+0.02f %+0.02f g %+0.02f %+0.02f %+0.02f d %+07.01f %+07.01f %+07.01f %+07.01f %+07.01f %+05.01f %04.01f a %+05.01f b %+05.01f q %+07.01f i %+05.01f t %+05.01f %04u %02u:%02u:%4.01f lla %+06.02f %+07.02f %06.01f v %+05.01f %+05.01f %+05.01f p %04.01f\n\r",
						mhp_sensors.static_pressure,
						mhp_sensors_gnss.magnetometer[0],
						mhp_sensors_gnss.magnetometer[1],
						mhp_sensors_gnss.magnetometer[2],
						mhp_sensors.accelerometer[0],
						mhp_sensors.accelerometer[1],
						mhp_sensors.accelerometer[2],
						mhp_sensors.gyroscope[0],
						mhp_sensors.gyroscope[1],
						mhp_sensors.gyroscope[2],
						mhp_sensors.dynamic_pressure[0],
						mhp_sensors.dynamic_pressure[1],
						mhp_sensors.dynamic_pressure[2],
						mhp_sensors.dynamic_pressure[3],
						mhp_sensors.dynamic_pressure[4],
						mhp_sensors.air_temperature,
						mhp_sensors.humidity,
						mhp_data.alpha*180.0/M_PI,
						mhp_data.beta*180.0/M_PI,
						mhp_data.q,
						mhp_data.ias,
						mhp_data.tas,
						mhp_sensors_gnss.week,
						mhp_sensors_gnss.hour,
						mhp_sensors_gnss.minute,
						mhp_sensors_gnss.seconds,
						mhp_sensors_gnss.latitude,
						mhp_sensors_gnss.longitude,
						mhp_sensors_gnss.altitude,
						mhp_sensors_gnss.velocity[0],
						mhp_sensors_gnss.velocity[1],
						mhp_sensors_gnss.velocity[2],
						mhp_sensors_gnss.pdop
							);

			if(write_file) {
				if(first_run) {
					sprintf(out,"%%"
							"STATIC_PRESSURE_TIME,"
							"STATIC_PRESSURE,"
							"MAGNETOMETER_TIME,"
							"MAGNETOMETER_X,"
							"MAGNETOMETER_Y,"
							"MAGNETOMETER_Z,"
							"IMU_TIME,"
							"ACCELEROMETER_X,"
							"ACCELEROMETER_Y,"
							"ACCELEROMETER_Z,"
							"GYROSCOPE_X,"
							"GYROSCOPE_Y,"
							"GYROSCOPE_Z,"
							"DYNAMIC_PRESSURE_TIME_0,"
							"DYNAMIC_PRESSURE_0,"
							"DYNAMIC_PRESSURE_TIME_1,"
							"DYNAMIC_PRESSURE_1,"
							"DYNAMIC_PRESSURE_TIME_2,"
							"DYNAMIC_PRESSURE_2,"
							"DYNAMIC_PRESSURE_TIME_3,"
							"DYNAMIC_PRESSURE_3,"
							"DYNAMIC_PRESSURE_TIME_4,"
							"DYNAMIC_PRESSURE_4,"
							"AIR_TEMPERATURE_TIME,"
							"AIR_TEMPERATURE,"
							"HUMIDITY_TIME,"
							"DATA_PRODUCT_TIME,"
							"ALPHA,"
							"BETA,"
							"Q,"
							"TAS,"
							"IAS,"
							"GPS_TIME,"
							"GPS_WEEK,"
							"HOUR,"
							"MINUTE,"
							"SECONDS,"
							"LATTIUDE,"
							"LONGITUDE,"
							"ALTITUDE,"
							"VELOCITY_N,"
							"VELOCITY_E,"
							"VELOCITY_D,"
							"PDOP"
							"\n");
					writeBytes((uint8_t*)out,strlen(out));
					first_run = 0;
				}

				sprintf(out,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
						mhp_timing.static_pressure_time,  // [s]
						mhp_sensors.static_pressure,
						mhp_timing.magnetometer_time, // [s]
						mhp_sensors_gnss.magnetometer[0],
						mhp_sensors_gnss.magnetometer[1],
						mhp_sensors_gnss.magnetometer[2],
						mhp_timing.imu_time, // [s]
						mhp_sensors.accelerometer[0],
						mhp_sensors.accelerometer[1],
						mhp_sensors.accelerometer[2],
						mhp_sensors.gyroscope[0],
						mhp_sensors.gyroscope[1],
						mhp_sensors.gyroscope[2],
						mhp_timing.dynamic_pressure_time[0],  // [s]
						mhp_sensors.dynamic_pressure[0],
						mhp_timing.dynamic_pressure_time[1],  // [s]
						mhp_sensors.dynamic_pressure[1],
						mhp_timing.dynamic_pressure_time[2],  // [s]
						mhp_sensors.dynamic_pressure[2],
						mhp_timing.dynamic_pressure_time[3],  // [s]
						mhp_sensors.dynamic_pressure[3],
						mhp_timing.dynamic_pressure_time[4],  // [s]
						mhp_sensors.dynamic_pressure[4],
						mhp_timing.air_temperature_time,  // [s]
						mhp_sensors.air_temperature,
						mhp_timing.humidity_time,  // [s]
						mhp_sensors.humidity,
						mhp_data.system_time,
						mhp_data.alpha,
						mhp_data.beta,
						mhp_data.q,
						mhp_data.ias,
						mhp_data.tas,
						mhp_timing.gps_time, // [s]
						(double)mhp_sensors_gnss.week,
						(double)mhp_sensors_gnss.hour,
						(double)mhp_sensors_gnss.minute,
						mhp_sensors_gnss.seconds,
						mhp_sensors_gnss.latitude,
						mhp_sensors_gnss.longitude,
						mhp_sensors_gnss.altitude,
						mhp_sensors_gnss.velocity[0],
						mhp_sensors_gnss.velocity[1],
						mhp_sensors_gnss.velocity[2],
						(double)mhp_sensors_gnss.pdop
							);

				writeBytes((uint8_t*)out,strlen(out));
			}

			break;

		case SENSORS_MHP_SENSORS:
			memcpy(&mhp_sensors,data,sizeof(MHPSensors_t));;
			break;

		case SENSORS_MHP_GNSS:
			memcpy(&mhp_sensors_gnss,data,sizeof(MHPSensorsGNSS_t));;
			break;

		case SENSORS_MHP_TIMING:
			memcpy(&mhp_timing,data,sizeof(MHPTiming_t));;

			if(display_telemetry_timing)
				printf("s %+0.02f d %+0.02f %+0.02f %+0.02f %+0.02f %+0.02f t %+0.02f h %+0.02f i %+0.02f m %+0.02f g %+0.02f\n\r",
						mhp_timing.static_pressure_time,  // [s]
						mhp_timing.dynamic_pressure_time[0],  // [s]
						mhp_timing.dynamic_pressure_time[1],  // [s]
						mhp_timing.dynamic_pressure_time[2],  // [s]
						mhp_timing.dynamic_pressure_time[3],  // [s]
						mhp_timing.dynamic_pressure_time[4],  // [s]
						mhp_timing.air_temperature_time,  // [s]
						mhp_timing.humidity_time,  // [s]
						mhp_timing.imu_time, // [s]
						mhp_timing.magnetometer_time, // [s]
						mhp_timing.gps_time // [s]
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
		case MAGNETOMETER:
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
