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
		float pressure, float temperature) {}

void updateStaticPressure(float system_time,
		float pressure, float temperature) {
	stat_p_cnt++;
}

void updateMHPSensors(float system_time,
		float static_pressure,
		float dynamic_pressure[5],
		float temperature,
		float humidity,
		float gyroscope[3],
		float accelerometer[3]) {}

void updateMHPRaw(float system_time,
		float differential_pressure[5]) {}

void updateMHPProducts(float system_time,
		float alpha,
		float beta,
		float ias,
		float tas) {}

void updateWind(float system_time, float u, float v, float w) {}

void updateHumidity(float system_time,
		float humidity) {}

void updateAGL(float system_time,
		float distance) {}

void updateProximity(float system_time,
		float distance) {}

void updateTemperature(float system_time,
		float temperature) {
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

void updateOrientation(float system_time,
		float q[4]) {}

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

void updateDeployTube(float system_time,
		uint8_t state,
		uint8_t parachute_door,
		uint8_t error) {}

void handleDeployTubeCmd(float system_time,
		uint8_t id,
		float value) {}

void printData() {
}
