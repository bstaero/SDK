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

typedef struct _MHPOld_t {
	uint8_t error_code;
	float system_time;  // [s]
	float static_pressure;  // [Pa]
	float dynamic_pressure[5];  // [Pa]
	float air_temperature;  // [deg C]
	float humidity;  // [%]
	float gyroscope[3];  // [rad/s]
	float accelerometer[3];  // [g]
	float magnetometer[3];  // [uT]
	float alpha;  // [rad]
	float beta;  // [rad]
	float q;  // [m/s]
	float ias;  // [m/s]
	float tas;  // [m/s]

#ifdef __cplusplus
	_MHPOld_t() {
		uint8_t _i;

		error_code = 0;
		system_time = 0.0;
		static_pressure = 0.0;

		for (_i = 0; _i < 5; ++_i)
			dynamic_pressure[_i] = 0.0;

		air_temperature = 0.0;
		humidity = 0.0;

		for (_i = 0; _i < 3; ++_i)
			gyroscope[_i] = 0.0;

		for (_i = 0; _i < 3; ++_i)
			accelerometer[_i] = 0.0;

		for (_i = 0; _i < 3; ++_i)
			magnetometer[_i] = 0.0;

		alpha = 0.0;
		beta = 0.0;
		q = 0.0;
		ias = 0.0;
		tas = 0.0;
	}
#endif
} __attribute__ ((packed)) MHPOld_t;


MHP_t            mhp_data;
MHPOld_t         mhp_old_data;
MHPSensors_t     mhp_sensors;
MHP9HSensors_t   mhp_9h_sensors;
MHPSensorsGNSS_t mhp_sensors_gnss;
MHPTiming_t      mhp_timing;
MHP9HTiming_t    mhp_9h_timing;

volatile SensorType_t calibration_requested = UNKNOWN_SENSOR;
volatile PacketTypes_t orientation_requested = (PacketTypes_t)0;
volatile PacketAction_t orientation_action = PKT_ACTION_NACK;


uint32_t product_cnt = 0;
uint32_t sensors_cnt = 0;

float last_print = 0;
/*<-End Global Variables-->*/

/*<---Local Functions----->*/
void handlePacket(uint8_t type, uint8_t action, const void * data, uint16_t size);
void printMHPValues(uint8_t num_ports);
/*<-End Local Functions--->*/

bool updateCommunications(void) {

	uint8_t data;
	bool retval = false;
	rx_packet.setAddressing(false);

	while(readByte(&data)) {
		retval = true;
		if(rx_packet.isValid(data)) {
			handlePacket(rx_packet.getType(), rx_packet.getAction(), rx_packet.getDataPtr(), rx_packet.getDataSize());
			break;
		}
	}

	return retval;
}

void handlePacket(uint8_t type, uint8_t action, const void * data, uint16_t size) 
{
	//printf("handlePacket: type=%u\n", type);

	PowerOn_t * power_on_data;
	CalibrateSensor_t * calibration_data;
	AxisMapping_t * axis_mapping;
	MHPOld_t * mhp_old;
	GPS_t * gps_old_data;

	switch(type) {

		case SENSORS_BOARD_ORIENTATION:
			switch(action) {
				case PKT_ACTION_STATUS:
					axis_mapping = (AxisMapping_t *)data;
					printf("IMU: x = [%i] y = [%i] z = [%i]\n",
							axis_mapping->axis[0],
							axis_mapping->axis[1],
							axis_mapping->axis[2]);

					break;

				case PKT_ACTION_ACK:
					orientation_action = PKT_ACTION_ACK;
					orientation_requested = (PacketTypes_t)0;
					break;

				case PKT_ACTION_NACK:
					orientation_action = PKT_ACTION_NACK;
					orientation_requested = (PacketTypes_t)0;
					break;
			}

			break;

		case SENSORS_GNSS_ORIENTATION:
			switch(action) {
				case PKT_ACTION_STATUS:
					axis_mapping = (AxisMapping_t *)data;
					printf("GNSS: x = [%i] y = [%i] z = [%i]\n",
							axis_mapping->axis[0],
							axis_mapping->axis[1],
							axis_mapping->axis[2]);
					break;

				case PKT_ACTION_ACK:
					orientation_action = PKT_ACTION_ACK;
					orientation_requested = (PacketTypes_t)0;
					break;

				case PKT_ACTION_NACK:
					orientation_action = PKT_ACTION_NACK;
					orientation_requested = (PacketTypes_t)0;
					break;
			}

			break;

		case SENSORS_CALIBRATE:
			calibration_data = (CalibrateSensor_t *)data;
			if(calibration_data->state == CALIBRATED)
				if(calibration_data->sensor == calibration_requested)
					calibration_requested = UNKNOWN_SENSOR;
			break;

		case SENSORS_MHP:
			product_cnt++;
			if(size == sizeof(MHP_t)) {
				memcpy(&mhp_data,data,sizeof(MHP_t));;
			}
			if(size == sizeof(MHPOld_t)) {
				mhp_old = (MHPOld_t *)data;

				mhp_data.system_time = mhp_old->system_time;
				mhp_data.alpha = mhp_old->alpha;
				mhp_data.beta = mhp_old->beta;
				mhp_data.q = mhp_old->q;
				mhp_data.ias = mhp_old->ias;
				mhp_data.tas = mhp_old->tas;

				mhp_sensors.system_time = mhp_old->system_time;
				mhp_sensors.error_code = mhp_old->error_code;
				mhp_sensors.static_pressure = mhp_old->static_pressure;
				mhp_sensors.dynamic_pressure[0] = mhp_old->dynamic_pressure[0];
				mhp_sensors.dynamic_pressure[1] = mhp_old->dynamic_pressure[1];
				mhp_sensors.dynamic_pressure[2] = mhp_old->dynamic_pressure[2];
				mhp_sensors.dynamic_pressure[3] = mhp_old->dynamic_pressure[3];
				mhp_sensors.dynamic_pressure[4] = mhp_old->dynamic_pressure[4];
				mhp_sensors.air_temperature = mhp_old->air_temperature;
				mhp_sensors.humidity = mhp_old->humidity;
				mhp_sensors.gyroscope[0] = mhp_old->gyroscope[0];
				mhp_sensors.gyroscope[1] = mhp_old->gyroscope[1];
				mhp_sensors.gyroscope[2] = mhp_old->gyroscope[2];
				mhp_sensors.accelerometer[0] = mhp_old->accelerometer[0];
				mhp_sensors.accelerometer[1] = mhp_old->accelerometer[1];
				mhp_sensors.accelerometer[2] = mhp_old->accelerometer[2];

				mhp_sensors_gnss.magnetometer[0] = mhp_old->magnetometer[0];
				mhp_sensors_gnss.magnetometer[1] = mhp_old->magnetometer[1];
				mhp_sensors_gnss.magnetometer[2] = mhp_old->magnetometer[2];

				printMHPValues(5);
			}

			break;

		case SENSORS_MHP_SENSORS:
			memcpy(&mhp_sensors,data,sizeof(MHPSensors_t));;
			sensors_cnt++;

			/*if(getElapsedTime() - last_print > 1.0) {
				last_print = getElapsedTime();
				printf("%0.2f %0.2f\n",
						(float)product_cnt/getElapsedTime(),
						(float)sensors_cnt/getElapsedTime());
			}*/

			printMHPValues(5);

			break;

		case SENSORS_MHP_9H_SENSORS:
			memcpy(&mhp_9h_sensors,data,sizeof(MHP9HSensors_t));;

			printMHPValues(9);

						break;

		case SENSORS_MHP_GNSS:
						//FIXME - this should not be sent by the GPS
						/*if( (((MHPSensorsGNSS_t *)data)-> latitude <  200.0) &&
								(((MHPSensorsGNSS_t *)data)-> latitude > -200.0) )
							memcpy(&mhp_sensors_gnss,data,sizeof(MHPSensorsGNSS_t));;*/

						memcpy(&mhp_sensors_gnss,data,sizeof(MHPSensorsGNSS_t));;
						break;

		case SENSORS_MHP_9H_TIMING:
						memcpy(&mhp_9h_timing,data,sizeof(MHP9HTiming_t));;

						if(display_telemetry_timing)
							printf("s %+0.02f d %+0.02f %+0.02f %+0.02f %+0.02f %+0.02f %+0.02f %+0.02f %+0.02f %+0.02f t %+0.02f h %+0.02f i %+0.02f m %+0.02f g %+0.02f\n\r",
									mhp_9h_timing.static_pressure_time,  // [s]
									mhp_9h_timing.dynamic_pressure_time[0],  // [s]
									mhp_9h_timing.dynamic_pressure_time[1],  // [s]
									mhp_9h_timing.dynamic_pressure_time[2],  // [s]
									mhp_9h_timing.dynamic_pressure_time[3],  // [s]
									mhp_9h_timing.dynamic_pressure_time[4],  // [s]
									mhp_9h_timing.dynamic_pressure_time[5],  // [s]
									mhp_9h_timing.dynamic_pressure_time[6],  // [s]
									mhp_9h_timing.dynamic_pressure_time[7],  // [s]
									mhp_9h_timing.dynamic_pressure_time[8],  // [s]
									mhp_9h_timing.air_temperature_time,  // [s]
									mhp_9h_timing.humidity_time,  // [s]
									mhp_9h_timing.imu_time, // [s]
									mhp_9h_timing.magnetometer_time, // [s]
									mhp_9h_timing.gps_time // [s]
									);
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

		case SENSORS_GPS:
						gps_old_data = (GPS_t *)data;

						mhp_sensors_gnss.system_time = gps_old_data->system_time;
						mhp_sensors_gnss.week =  gps_old_data->week;
						mhp_sensors_gnss.hour =  gps_old_data->hour;
						mhp_sensors_gnss.minute =  gps_old_data->minute;
						mhp_sensors_gnss.seconds =  gps_old_data->seconds;
						mhp_sensors_gnss.latitude = gps_old_data->latitude;
						mhp_sensors_gnss.longitude = gps_old_data->longitude;
						mhp_sensors_gnss.altitude = gps_old_data->altitude;
						mhp_sensors_gnss.velocity[0] = gps_old_data->velocity.x;
						mhp_sensors_gnss.velocity[1] = gps_old_data->velocity.y;
						mhp_sensors_gnss.velocity[2] = gps_old_data->velocity.z;
						mhp_sensors_gnss.pdop = gps_old_data->pdop;
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
	tx_packet.setAction(PKT_ACTION_COMMAND);
	tx_packet.setData((uint8_t *)&calibrate_pkt, sizeof(CalibrateSensor_t));

	writeBytes(tx_packet.getPacket(), tx_packet.getSize());
}

void requestPowerOn(void) {
	PowerOn_t power_on_pkt;

	tx_packet.setAddressing(false);
	tx_packet.setType(SYSTEM_POWER_ON);
	tx_packet.setAction(PKT_ACTION_REQUEST);
	tx_packet.setData((uint8_t *)&power_on_pkt, sizeof(PowerOn_t));

	writeBytes(tx_packet.getPacket(), tx_packet.getSize());
}

void requestOrientation(PacketTypes_t type) {
	if(type != SENSORS_BOARD_ORIENTATION && type != SENSORS_GNSS_ORIENTATION)
		return;

	AxisMapping_t axis_mapping_pkt;

	tx_packet.setAddressing(false);
	tx_packet.setType(type);
	tx_packet.setAction(PKT_ACTION_REQUEST);
	tx_packet.setData((uint8_t *)&axis_mapping_pkt, sizeof(AxisMapping_t));

	writeBytes(tx_packet.getPacket(), tx_packet.getSize());
}

void setOrientation(PacketTypes_t type, AxisMapping_t * axis_mapping) {
	if(orientation_requested) return;

	if(type != SENSORS_BOARD_ORIENTATION && type != SENSORS_GNSS_ORIENTATION)
		return;

	orientation_action = PKT_ACTION_NACK;
	orientation_requested = type;

	AxisMapping_t axis_mapping_pkt;

	tx_packet.setAddressing(false);
	tx_packet.setType(type);
	tx_packet.setAction(PKT_ACTION_COMMAND);
	tx_packet.setData((uint8_t *)axis_mapping, sizeof(AxisMapping_t));

	writeBytes(tx_packet.getPacket(), tx_packet.getSize());
}

void printMHPValues(uint8_t num_ports) {
	static bool first_run = true;
	char out[2048];

	if(num_ports == 5) {
		if(display_telemetry)
			printf("%+0.01f m %+05.01f %+05.01f %+05.01f a %+0.02f %+0.02f %+0.02f g %+0.02f %+0.02f %+0.02f d %+07.01f %+07.01f %+07.01f %+07.01f %+07.01f %+05.01f %04.01f a %+05.01f b %+05.01f q %+07.01f i %+05.01f t %+05.01f %04u %02u:%02u:%04.01f lla %+06.02f %+07.02f %06.01f v %+05.01f %+05.01f %+05.01f p %04.01f\n\r",
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
							"HUMIDITY,"
							"DATA_PRODUCT_TIME,"
							"ALPHA,"
							"BETA,"
							"Q,"
							"TAS,"
							"IAS,"
							"U,"
							"V,"
							"W,"
							"Q_0,"
							"Q_1,"
							"Q_2,"
							"Q_3,"
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
					writeFile((uint8_t*)out,strlen(out));
					first_run = 0;
				}

				sprintf(out,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
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
						mhp_data.wind[0],
						mhp_data.wind[1],
						mhp_data.wind[2],
						mhp_data.quaternion[0],
						mhp_data.quaternion[1],
						mhp_data.quaternion[2],
						mhp_data.quaternion[3],
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

						writeFile((uint8_t*)out,strlen(out));
			}
	}

	if(num_ports == 9) {
		if(display_telemetry)
			printf("%+0.01f m %+05.01f %+05.01f %+05.01f a %+0.02f %+0.02f %+0.02f g %+0.02f %+0.02f %+0.02f d %+07.01f %+07.01f %+07.01f %+07.01f %+07.01f %+07.01f %+07.01f %+07.01f %+07.01f %+05.01f %04.01f a %+05.01f b %+05.01f q %+07.01f i %+05.01f t %+05.01f %04u %02u:%02u:%04.01f lla %+06.02f %+07.02f %06.01f v %+05.01f %+05.01f %+05.01f p %04.01f\n\r",
					mhp_9h_sensors.static_pressure,
					mhp_sensors_gnss.magnetometer[0],
					mhp_sensors_gnss.magnetometer[1],
					mhp_sensors_gnss.magnetometer[2],
					mhp_9h_sensors.accelerometer[0],
					mhp_9h_sensors.accelerometer[1],
					mhp_9h_sensors.accelerometer[2],
					mhp_9h_sensors.gyroscope[0],
					mhp_9h_sensors.gyroscope[1],
					mhp_9h_sensors.gyroscope[2],
					mhp_9h_sensors.dynamic_pressure[0],
					mhp_9h_sensors.dynamic_pressure[1],
					mhp_9h_sensors.dynamic_pressure[2],
					mhp_9h_sensors.dynamic_pressure[3],
					mhp_9h_sensors.dynamic_pressure[4],
					mhp_9h_sensors.dynamic_pressure[5],
					mhp_9h_sensors.dynamic_pressure[6],
					mhp_9h_sensors.dynamic_pressure[7],
					mhp_9h_sensors.dynamic_pressure[8],
					mhp_9h_sensors.air_temperature,
					mhp_9h_sensors.humidity,
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
							"DYNAMIC_PRESSURE_TIME_5,"
							"DYNAMIC_PRESSURE_5,"
							"DYNAMIC_PRESSURE_TIME_6,"
							"DYNAMIC_PRESSURE_6,"
							"DYNAMIC_PRESSURE_TIME_7,"
							"DYNAMIC_PRESSURE_7,"
							"DYNAMIC_PRESSURE_TIME_8,"
							"DYNAMIC_PRESSURE_8,"
							"AIR_TEMPERATURE_TIME,"
							"AIR_TEMPERATURE,"
							"HUMIDITY_TIME,"
							"HUMIDITY,"
							"DATA_PRODUCT_TIME,"
							"ALPHA,"
							"BETA,"
							"Q,"
							"TAS,"
							"IAS,"
							"U,"
							"V,"
							"W,"
							"Q_0,"
							"Q_1,"
							"Q_2,"
							"Q_3,"
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
					writeFile((uint8_t*)out,strlen(out));
					first_run = 0;
				}

				sprintf(out,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
						mhp_9h_timing.static_pressure_time,  // [s]
						mhp_9h_sensors.static_pressure,
						mhp_9h_timing.magnetometer_time, // [s]
						mhp_sensors_gnss.magnetometer[0],
						mhp_sensors_gnss.magnetometer[1],
						mhp_sensors_gnss.magnetometer[2],
						mhp_9h_timing.imu_time, // [s]
						mhp_9h_sensors.accelerometer[0],
						mhp_9h_sensors.accelerometer[1],
						mhp_9h_sensors.accelerometer[2],
						mhp_9h_sensors.gyroscope[0],
						mhp_9h_sensors.gyroscope[1],
						mhp_9h_sensors.gyroscope[2],
						mhp_9h_timing.dynamic_pressure_time[0],  // [s]
						mhp_9h_sensors.dynamic_pressure[0],
						mhp_9h_timing.dynamic_pressure_time[1],  // [s]
						mhp_9h_sensors.dynamic_pressure[1],
						mhp_9h_timing.dynamic_pressure_time[2],  // [s]
						mhp_9h_sensors.dynamic_pressure[2],
						mhp_9h_timing.dynamic_pressure_time[3],  // [s]
						mhp_9h_sensors.dynamic_pressure[3],
						mhp_9h_timing.dynamic_pressure_time[4],  // [s]
						mhp_9h_sensors.dynamic_pressure[4],
						mhp_9h_timing.dynamic_pressure_time[5],  // [s]
						mhp_9h_sensors.dynamic_pressure[5],
						mhp_9h_timing.dynamic_pressure_time[6],  // [s]
						mhp_9h_sensors.dynamic_pressure[6],
						mhp_9h_timing.dynamic_pressure_time[7],  // [s]
						mhp_9h_sensors.dynamic_pressure[7],
						mhp_9h_timing.dynamic_pressure_time[8],  // [s]
						mhp_9h_sensors.dynamic_pressure[8],
						mhp_9h_timing.air_temperature_time,  // [s]
						mhp_9h_sensors.air_temperature,
						mhp_9h_timing.humidity_time,  // [s]
						mhp_9h_sensors.humidity,
						mhp_data.system_time,
						mhp_data.alpha,
						mhp_data.beta,
						mhp_data.q,
						mhp_data.ias,
						mhp_data.tas,
						mhp_data.wind[0],
						mhp_data.wind[1],
						mhp_data.wind[2],
						mhp_data.quaternion[0],
						mhp_data.quaternion[1],
						mhp_data.quaternion[2],
						mhp_data.quaternion[3],
						mhp_9h_timing.gps_time, // [s]
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

						writeFile((uint8_t*)out,strlen(out));
			}
	}
}
