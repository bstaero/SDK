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

#include "can_handler.h"
#include "test_handler.h"

#include "test.h"
#include "main.h"
#include "structs.h"

#include "canpackets.h"

using namespace bst::comms::canpackets;

/*<---Global Variables---->*/
uint32_t gps_count;
uint32_t imu_count;
uint32_t pressure_count;
uint32_t pwm_in_count;

uint8_t can_pressure_sensor;
uint8_t can_imu_sensor;
uint8_t can_mag_sensor;
uint8_t can_gps_sensor;

uint8_t can_pwm_in;

extern MHP_t            mhp_data;
extern MHPSensors_t     mhp_sensors;
extern MHP9HSensors_t   mhp_9h_sensors;
extern MHPSensorsGNSS_t mhp_sensors_gnss;
extern MHPTiming_t      mhp_timing;
extern MHP9HTiming_t    mhp_9h_timing;
/*<-End Global Variables-->*/

/*<---Local Functions----->*/
void updateActuatorValues(uint16_t * values) {}

void updatePWMIn(float system_time, uint16_t * usec) {}

void updateGPS(
		float system_time, uint16_t week, uint8_t hour, uint8_t minute, float seconds,
		double latitude, double longitude, float altitude,
		float vel_n, float vel_e, float vel_d,
		float course, float speed,
		float pdop, uint8_t satellites, uint8_t fix_type) {}

void updateAccelerometer(float system_time,
		float ax, float ay, float az) {
	mhp_sensors.system_time = system_time;
	mhp_sensors.accelerometer[0] = ax;
	mhp_sensors.accelerometer[1] = ay;
	mhp_sensors.accelerometer[2] = az;
}

void updateGyroscope(float system_time,
		float gx, float gy, float gz) {
	mhp_sensors.system_time = system_time;
	mhp_sensors.gyroscope[0] = gx;
	mhp_sensors.gyroscope[1] = gy;
	mhp_sensors.gyroscope[2] = gz;
}

void updateMagnetometer(float system_time,
		float mx, float my, float mz) {
	mhp_sensors_gnss.system_time = system_time;
	mhp_sensors_gnss.magnetometer[0] = mx;
	mhp_sensors_gnss.magnetometer[1] = my;
	mhp_sensors_gnss.magnetometer[2] = mz;
}

void updateIMU(float system_time, 
		float ax, float ay, float az, 
		float gx, float gy, float gz, 
		float mx, float my, float mz) {}

void updateDynamicPressure(float system_time,
		float pressure, float temperature) {}

void updateStaticPressure(float system_time,
		float pressure, float temperature) {
}


void updateAirData(float system_time,
		float static_pressure, // [Pa]
		float dynamic_pressure, // [Pa]
		float air_temperature, // [deg C]
		float humidity) {  // [%]
	mhp_sensors.system_time = system_time;
	mhp_sensors.static_pressure = static_pressure;
	mhp_data.q = dynamic_pressure;
	mhp_sensors.air_temperature = air_temperature;
	mhp_sensors.humidity = humidity;
}

void updateMHPSensors(float system_time,
		float static_pressure,
		float dynamic_pressure[5],
		float temperature,
		float humidity,
		float gyroscope[3],
		float accelerometer[3]) {}

void updateMHPRaw(float system_time,
		float differential_pressure[5]) {
	mhp_sensors.system_time = system_time;
	mhp_sensors.dynamic_pressure[0] = differential_pressure[0];
	mhp_sensors.dynamic_pressure[1] = differential_pressure[1];
	mhp_sensors.dynamic_pressure[2] = differential_pressure[2];
	mhp_sensors.dynamic_pressure[3] = differential_pressure[3];
	mhp_sensors.dynamic_pressure[4] = differential_pressure[4];
}

void updateMHPProducts(float system_time,
		float alpha,
		float beta,
		float ias,
		float tas) {

	mhp_data.system_time = system_time;
	mhp_data.alpha = alpha;
	mhp_data.beta = beta;
	mhp_data.ias = ias;
	mhp_data.tas = tas;
	
	printMHPValues(5);
}

void updateWind(float system_time, float u, float v, float w) {
	mhp_data.system_time = system_time;
	mhp_data.wind[0] = u;
	mhp_data.wind[1] = v;
	mhp_data.wind[2] = w;
}

void updateHumidity(float system_time,
		float humidity) {}

void updateAGL(float system_time,
		float distance) {}

void updateProximity(float system_time,
		float distance) {}

void updateTemperature(float system_time,
		float temperature) {
}

void updatePowerOn(uint16_t comms_rev, uint32_t serial_num) {
	printf("serial number: 0x%x  comm revision: %u\n",
			serial_num,
			comms_rev
			);
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
		float ts, uint16_t w, uint8_t h, uint8_t m, float s) {
	mhp_sensors_gnss.system_time = ts;
	mhp_sensors_gnss.week = w;
	mhp_sensors_gnss.hour = h;
	mhp_sensors_gnss.minute = m;
	mhp_sensors_gnss.seconds = s;
}


void updateGPSLLAValues(
		float ts,
		double latitude, double longitude, float altitude) {
	mhp_sensors_gnss.system_time = ts;
	mhp_sensors_gnss.latitude = latitude;
	mhp_sensors_gnss.longitude = longitude;
	mhp_sensors_gnss.altitude = altitude;
}

void updateGPSVelValues(
		float ts,
		float course, float sog,
		float vel_n, float vel_e, float vel_d) {
	mhp_sensors_gnss.system_time = ts;
	//FIXME -- need to check NED vs XYZ everywhere in system
	mhp_sensors_gnss.velocity[0] = vel_n;
	mhp_sensors_gnss.velocity[1] = vel_e;
	mhp_sensors_gnss.velocity[2] = vel_d;
}

void updateGPSHealthValues(
		float ts,
		float pdop, uint8_t satellites, uint8_t fix_type) {
	mhp_sensors_gnss.pdop = pdop;
}

void updateMagValues(
		float ts,
		float mag_x,
		float mag_y,
		float mag_z) {
	mhp_sensors_gnss.system_time = ts;
	mhp_sensors_gnss.magnetometer[0] = mag_x;
	mhp_sensors_gnss.magnetometer[1] = mag_y;
	mhp_sensors_gnss.magnetometer[2] = mag_z;
}

void updateOrientation(float system_time,
		float q[4]) {
	mhp_data.system_time = system_time;
	mhp_data.quaternion[0] = q[0];
	mhp_data.quaternion[1] = q[1];
	mhp_data.quaternion[2] = q[2];
	mhp_data.quaternion[3] = q[3];
}

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

void updateCalibration(CAN_SensorType_t sensor,
		CAN_CalibrationState_t state) {

	if(state == CAN_CALIBRATED)
		if(sensor == calibration_requested)
			calibration_requested = UNKNOWN_SENSOR;
}

void updateBoardAxis(int8_t axis[3]) {
	printf("IMU: x = [%i] y = [%i] z = [%i]\n",
			axis[0],
			axis[1],
			axis[2]);
}

void updateGNSSAxis(int8_t axis[3]) {
	printf("GNSS: x = [%i] y = [%i] z = [%i]\n",
			axis[0],
			axis[1],
			axis[2]);
}

void updatePayloadTrigger(float system_time,
		uint16_t id, uint8_t channel) {}
/*<-End Local Functions--->*/


