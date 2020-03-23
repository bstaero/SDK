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
#ifndef _TEST_HANDLER_H_
#define _TEST_HANDLER_H_

#include <inttypes.h>
#include "bst_packet.h"

extern volatile SensorType_t calibration_requested;

void sendCalibrate(SensorType_t sensor);
void requestPowerOn(void);
void updateCommunications(void);

typedef struct _MHP_Time_t {
	uint8_t error_code;
	uint8_t padding_0;
	uint8_t padding_1;
	uint8_t padding_2;
	float static_pressure_time;  // [s]
	float static_pressure;  // [Pa]
	float dynamic_pressure_time[5];  // [s]
	float dynamic_pressure[5];  // [Pa]
	float air_temperature_time;  // [s]
	float air_temperature;  // [deg C]
	float humidity_time;  // [s]
	float humidity;  // [%]
	float imu_time; // [s]
	float gyroscope[3];  // [rad/s]
	float accelerometer[3];  // [g]
	float magnetometer_time; // [s]
	float magnetometer[3];  // [uT]
	float alpha;  // [rad]
	float beta;  // [rad]
	float q;  // [m/s]
	float ias;  // [m/s]
	float tas;  // [m/s]

#ifdef __cplusplus
	_MHP_Time_t() {
		uint8_t _i;

		error_code = 0;
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
} __attribute__ ((packed)) MHP_Time_t;

#endif
