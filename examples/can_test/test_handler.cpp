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

uint32_t gps_count;
uint32_t imu_count;
uint32_t pressure_count;
uint32_t pwm_in_count;

uint8_t can_pressure_sensor;
uint8_t can_imu_sensor;
uint8_t can_mag_sensor;
uint8_t can_gps_sensor;

uint8_t can_pwm_in;


uint32_t gnss_lla_cnt;
uint32_t gnss_utc_cnt;
uint32_t gnss_vel_cnt;
uint32_t gnss_hs_cnt;

uint32_t mag_cnt;

uint32_t stat_p_cnt;

uint8_t p_new_gps_data;

float local_dynamic_pressure = 0.0;
float local_static_pressure = 0.0;
float local_temperature = 0.0;
float local_humidity = 0.0;

float local_data[8];

/*<---Local Functions----->*/
void printData(void);
/*<-End Local Functions--->*/


void updateActuatorValues(uint16_t * values) { 
	printf("Act:");
	for(uint8_t i=0; i<16; i++) printf(" [%04u]",values[i]);
	printf("\n");
}
void updatePWMIn(float system_time, uint16_t * usec) {}

void updateGPS(
		float system_time, uint16_t week, uint8_t hour, uint8_t minute, float seconds,
		double latitude, double longitude, float altitude,
		float vel_n, float vel_e, float vel_d,
		float course, float speed,
		float pdop, uint8_t satellites, uint8_t fix_type) {}

void updateAccelerometer(float system_time,
		float ax, float ay, float az) {}

void updateGyroscope(float system_time,
		float gx, float gy, float gz) {}

void updateMagnetometer(float system_time,
		float mx, float my, float mz) {mag_cnt++;}

void updateIMU(float system_time, 
		float ax, float ay, float az, 
		float gx, float gy, float gz, 
		float mx, float my, float mz) {}

void updateDynamicPressure(float system_time,
		float pressure, float temperature) {
	local_dynamic_pressure = pressure;
}

void updateStaticPressure(float system_time,
		float pressure, float temperature) {
	stat_p_cnt++;
	local_static_pressure = pressure;
}

void updateMHPSensors(float system_time,
		float static_pressure,
		float dynamic_pressure[5],
		float temperature,
		float humidity,
		float gyroscope[3],
		float accelerometer[3]) {
	printf("MHP\n");
}

void updateHumidity(float system_time,
		float humidity) {
	local_humidity = humidity;

	local_data[0] = getElapsedTime();
	local_data[1] = local_dynamic_pressure;
	local_data[2] = local_static_pressure;
	local_data[3] = local_temperature;
	local_data[4] = local_humidity;
}

void updateAGL(float system_time,
		float distance) {}

void updateProximity(float system_time,
		float distance) {}

void updateTemperature(float system_time,
		float temperature) {
	if(temperature != 0.0) local_temperature = temperature;
	else {

		local_data[5] = getElapsedTime();
		local_data[6] = local_dynamic_pressure;
		local_data[7] = local_static_pressure;

		printData();
	}
}

void updateSupply(float system_time,
		float voltage, float current, float coulomb_count, float temperature) {}

void updateGPSValues(
		float ts, int16_t w, uint8_t h, uint8_t m, float s,
		double latitude, double longitude, float altitude,
		float vel_n, float vel_e, float vel_d,
		float course, float sog,
		float pdop, uint8_t satellites) {}


void updateGPSUTCValues(
		float ts, uint16_t w, uint8_t h, uint8_t m, float s) {gnss_utc_cnt++;}


void updateGPSLLAValues(
		float ts,
		double latitude, double longitude, float altitude) {gnss_lla_cnt++;}

void updateGPSVelValues(
		float ts,
		float course, float sog,
		float vel_n, float vel_e, float vel_d) {gnss_vel_cnt++;}

void updateGPSHealthValues(
		float ts,
		float pdop, uint8_t satellites, uint8_t fix_type) {gnss_hs_cnt++;}

void updateMagValues(
		float ts,
		float mag_x,
		float mag_y,
		float mag_z) {}

void handleNDVI(float ts, uint8_t id, float red, float near_ir, float ir_ambient, float ir_object) {}

void updateGPSRTCM(float t0, uint8_t size, uint8_t * data) {}

void updateGPSSVIN(
		uint32_t time_elapsed,
		uint32_t time_minimum,
		float accuracy,
		float accuracy_minimum,
		uint8_t flags) {}

void updateADSB(float system_time,
		uint32_t icao_address,
		double latitude,
		double longitude,
		uint8_t altitude_type,
		float altitude,
		float heading,
		float horizontal_velocity,
		float vertical_velocity,
		char callsign[9],
		uint8_t emitter_type,
		uint8_t tslc,
		uint16_t flags,
		uint16_t squawk) {}

void updatePayloadTrigger(float system_time,
		uint16_t id, uint8_t channel) {}


void printData() {
	static bool first_run = true;
	char out[2048];

		if(display_telemetry)
			printf("%07.03f sec %+06.01f Pa %+07.01f Pa %+05.01f deg C %04.01f %% | %07.03f sec %+06.01f Pa %+07.01f Pa\n\r",
					local_data[0],
					local_data[1],
					local_data[2],
					local_data[3],
					local_data[4],
					local_data[5],
					local_data[6],
					local_data[7]
						);

			if(write_file) {
				if(first_run) {
					sprintf(out,"%%"
							"PRESSURE_0_TIME,"
							"DYNAMIC_PRESSURE_0,"
							"STATIC_PRESSURE_0,"
							"DYNAMIC_PRESSURE_TIME_0,"
							"AIR_TEMPERATURE,"
							"HUMIDITY,"
							"DYNAMIC_PRESSURE_1,"
							"STATIC_PRESSURE_1,"
							"\n");
					writeFile((uint8_t*)out,strlen(out));
					first_run = 0;
				}

				sprintf(out,"%f,%f,%f,%f,%f,%f,%f,%f\n", 
					local_data[0],
					local_data[1],
					local_data[2],
					local_data[3],
					local_data[4],
					local_data[5],
					local_data[6],
					local_data[7]
							);

						writeFile((uint8_t*)out,strlen(out));
			}
}
