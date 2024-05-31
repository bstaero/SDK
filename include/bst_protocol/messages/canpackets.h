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

                               Jack Elston
                       Jack.Elston@blackswifttech.com                          

                               Cory Dixon
                         Cory.Dixon@blackswifttech.com                          

                             Maciej Stachura
|                   Maciej.Stachura@blackswifttech.com                         |
|                                                                              |
\*=+--+=#=+--                 --+=#=+--+=#=+--                    --+=#=+--+=#*/

/*                     THIS FILE IS AUTOGENERATED BY                          *\
|*                                msg-gen.py                                  *|
\*                               DO NOT EDIT                                  */

#ifndef _CANPACKETS_H_
#define _CANPACKETS_H_

#include <inttypes.h>

#ifdef __cplusplus
namespace bst {
namespace comms {
namespace canpackets {
#endif

/*--------[ Actuators ]--------*/

#define CAN_NUM_ACTUATORS 16

/*--------[ Comms ]--------*/

typedef enum {
	/* SENSORS */
	CAN_PKT_PRESSURE=16,
	CAN_PKT_AIR_DATA=17,
	CAN_PKT_MHP=18,
	CAN_PKT_MHP_RAW=19,
	CAN_PKT_MHP_PRODUCTS=20,
	CAN_PKT_WIND=21,
	CAN_PKT_IMU=32,
	CAN_PKT_ACCEL=33,
	CAN_PKT_GYRO=34,
	CAN_PKT_MAG=35,
	CAN_PKT_ORIENTATION=36,
	CAN_PKT_GNSS=48,
	CAN_PKT_GNSS_UTC=49,
	CAN_PKT_GNSS_LLA=50,
	CAN_PKT_GNSS_VEL=51,
	CAN_PKT_GNSS_HEALTH=52,
	CAN_PKT_GNSS_UTC_W=53,
	CAN_PKT_GNSS_RTCM=54,
	CAN_PKT_GNSS_HEALTH_2=55,
	CAN_PKT_GNSS_SVIN=56,
	CAN_PKT_AGL=96,
	CAN_PKT_PROXIMITY=112,
	CAN_PKT_ADSB=144,
	CAN_PKT_CALIBRATE=160,
	CAN_PKT_BOARD_ORIENTATION=161,
	CAN_PKT_GNSS_ORIENTATION=162,

	/* STATE */

	/* CONTROL */
	CAN_PKT_DEPLOYMENT_TUBE_CMD=129,

	/* ACTUATORS */
	CAN_PKT_ACTUATOR=2047,

	/* INPUT */
	CAN_PKT_RECEIVER=1,

	/* SYSTEM */
	CAN_PKT_SUPPLY=64,
	CAN_PKT_POWER_ON=65,

	/* TELEMETRY */
	CAN_PKT_DEPLOYMENT_TUBE=128,

	/* HWIL */

	/* FLIGHT PLAN */

	/* VEHICLE CONFIGURATION */

	/* MISSION */

	/* PAYLOAD */
	CAN_PKT_NDVI=80,
	CAN_PKT_NDVI_DOWN=81,
	CAN_PKT_NDVI_UP=82,
	CAN_PKT_TRIGGER=83,

	/* ERRORS */
}  __attribute__ ((packed)) CAN_PacketTypes_t;

/*--------[ Control ]--------*/

typedef enum {
	DEPLOY_TUBE_INIT,
	DEPLOY_TUBE_READY,
	DEPLOY_TUBE_ARMED,
	DEPLOY_TUBE_FLAP_OPEN,
	DEPLOY_TUBE_PARA_DEPLOYED,
	DEPLOY_TUBE_JETTISONED,
	DEPLOY_TUBE_AC_RELASED,
	DEPLOY_TUBE_SHUTDOWN,
	DEPLOY_TUBE_ERROR,
}  __attribute__ ((packed)) CAN_DeploymentTubeState_t;

typedef struct _CAN_DeploymentTubeCommand_t {
	uint8_t startByte;

	uint8_t id;
	float value;

	uint16_t chk;

#ifdef __cplusplus
	_CAN_DeploymentTubeCommand_t() {
		startByte = 0;
		id = 255;
		value = 0.0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_DeploymentTubeCommand_t;

/*--------[ PAYLOAD ]--------*/

typedef struct _CAN_NDVI_t {
	uint8_t startByte;

	uint8_t id;
	float red;
	float near_ir;
	float ir_ambient;
	float ir_object;

	uint16_t chk;

#ifdef __cplusplus
	_CAN_NDVI_t() {
		startByte = 0;
		id = 0;
		red = 0.0;
		near_ir = 0.0;
		ir_ambient = 0.0;
		ir_object = 0.0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_NDVI_t;

/*--------[ SENSORS ]--------*/

typedef struct _CAN_ADSB_t {
	uint8_t startByte;

	float timestamp;
	uint32_t icao_address;
	double latitude;
	double longitude;
	uint8_t altitude_type;
	float altitude;
	float heading;
	float horizontal_velocity;
	float vertical_velocity;
	char callsign[9];
	uint8_t emitter_type;
	uint8_t tslc;
	uint16_t flags;
	uint16_t squawk;

	uint16_t chk;

#ifdef __cplusplus
	_CAN_ADSB_t() {
		uint8_t _i;

		startByte = 0;
		timestamp = 0.0;
		icao_address = 0;
		latitude = 0.0;
		longitude = 0.0;
		altitude_type = 0;
		altitude = 0.0;
		heading = 0.0;
		horizontal_velocity = 0.0;
		vertical_velocity = 0.0;

		for (_i = 0; _i < 9; ++_i)
			callsign[_i] = 0;

		emitter_type = 0;
		tslc = 0;
		flags = 0;
		squawk = 0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_ADSB_t;

typedef struct _CAN_AGL_t {
	uint8_t startByte;

	float timestamp;
	float distance;
	float velocity;

	uint16_t chk;

#ifdef __cplusplus
	_CAN_AGL_t() {
		startByte = 0;
		timestamp = 0.0;
		distance = 0.0;
		velocity = 0.0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_AGL_t;

typedef struct _CAN_Accelerometer_t {
	uint8_t startByte;

	float ax;  // [g]
	float ay;  // [g]
	float az;  // [g]
	float temp;  // [deg C]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_Accelerometer_t() {
		startByte = 0;
		ax = 0.0;
		ay = 0.0;
		az = 0.0;
		temp = 0.0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_Accelerometer_t;

typedef struct _CAN_AirData_t {
	uint8_t startByte;

	float static_pressure;  // [Pa]
	float dynamic_pressure;  // [Pa]
	float air_temperature;  // [deg C]
	float humidity;  // [%]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_AirData_t() {
		startByte = 0;
		static_pressure = 0.0;
		dynamic_pressure = 0.0;
		air_temperature = 0.0;
		humidity = 0.0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_AirData_t;

typedef struct _CAN_GNSS_t {
	uint8_t startByte;

	uint16_t week;
	uint8_t hours;  // [hrs]
	uint8_t minutes;  // [min]
	float seconds;  // [sec]
	double latitude;  // [deg]
	double longitude;  // [deg]
	float altitude;  // [m]
	float heading;  // [rad]
	float speed;  // [m/s]
	float pdop;  // [-]
	uint8_t satellites;  // [-]
	uint8_t fix_type;  // [-]
	float vx;  // [m/s]
	float vy;  // [m/s]
	float vz;  // [m/s]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_GNSS_t() {
		startByte = 0;
		week = 0;
		hours = 0;
		minutes = 0;
		seconds = 0.0;
		latitude = 0.0;
		longitude = 0.0;
		altitude = 0.0;
		heading = 0.0;
		speed = 0.0;
		pdop = 0.0;
		satellites = 0;
		fix_type = 0;
		vx = 0.0;
		vy = 0.0;
		vz = 0.0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_GNSS_t;

typedef struct _CAN_GNSS_HEALTH_t {
	uint8_t startByte;

	float pdop;  // [-]
	uint8_t satellites;  // [-]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_GNSS_HEALTH_t() {
		startByte = 0;
		pdop = 0.0;
		satellites = 0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_GNSS_HEALTH_t;

typedef struct _CAN_GNSS_HEALTH_2_t {
	uint8_t startByte;

	float pdop;  // [-]
	uint8_t satellites;  // [-]
	uint8_t fix_type;  // [-]
	uint8_t buffer[2];  // [-]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_GNSS_HEALTH_2_t() {
		uint8_t _i;

		startByte = 0;
		pdop = 0.0;
		satellites = 0;
		fix_type = 0;

		for (_i = 0; _i < 2; ++_i)
			buffer[_i] = 0;

		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_GNSS_HEALTH_2_t;

typedef struct _CAN_GNSS_VEL_t {
	uint8_t startByte;

	float heading;  // [rad]
	float speed;  // [m/s]
	float vx;  // [m/s]
	float vy;  // [m/s]
	float vz;  // [m/s]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_GNSS_VEL_t() {
		startByte = 0;
		heading = 0.0;
		speed = 0.0;
		vx = 0.0;
		vy = 0.0;
		vz = 0.0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_GNSS_VEL_t;

typedef struct _CAN_Gyroscope_t {
	uint8_t startByte;

	float gx;  // [rad/s]
	float gy;  // [rad/s]
	float gz;  // [rad/s]
	float temp;  // [deg C]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_Gyroscope_t() {
		startByte = 0;
		gx = 0.0;
		gy = 0.0;
		gz = 0.0;
		temp = 0.0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_Gyroscope_t;

typedef struct _CAN_IMU_t {
	uint8_t startByte;

	float ax;  // [g]
	float ay;  // [g]
	float az;  // [g]
	float gx;  // [rad/s]
	float gy;  // [rad/s]
	float gz;  // [rad/s]
	float mx;  // [uT]
	float my;  // [uT]
	float mz;  // [uT]
	float temp;  // [deg C]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_IMU_t() {
		startByte = 0;
		ax = 0.0;
		ay = 0.0;
		az = 0.0;
		gx = 0.0;
		gy = 0.0;
		gz = 0.0;
		mx = 0.0;
		my = 0.0;
		mz = 0.0;
		temp = 0.0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_IMU_t;

typedef struct _CAN_MHP_t {
	uint8_t startByte;

	/* uint8 error_code */
	uint32_t system_time;  // [ms]
	uint32_t static_pressure;  // [mPa]
	int32_t dynamic_pressure[5];  // [mPa]
	int16_t air_temperature;  // [deg C*100]
	uint16_t humidity;  // [%*100]
	int16_t gyroscope[3];  // [mrad/s]
	int16_t accelerometer[3];  // [mG]
	int16_t magnetometer[3];  // [nT/10]
	int16_t alpha;  // [mrad]
	int16_t beta;  // [mrad]
	/* float q  # [Pa] */
	/* float ias  # [m/s] */
	/* float tas  # [m/s] */

	uint16_t chk;

#ifdef __cplusplus
	_CAN_MHP_t() {
		uint8_t _i;

		startByte = 0;
		system_time = 0;
		static_pressure = 0;

		for (_i = 0; _i < 5; ++_i)
			dynamic_pressure[_i] = 0;

		air_temperature = 0;
		humidity = 0;

		for (_i = 0; _i < 3; ++_i)
			gyroscope[_i] = 0;

		for (_i = 0; _i < 3; ++_i)
			accelerometer[_i] = 0;

		for (_i = 0; _i < 3; ++_i)
			magnetometer[_i] = 0;

		alpha = 0;
		beta = 0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_MHP_t;

typedef struct _CAN_MHP_Products_t {
	uint8_t startByte;

	float alpha;  // [rad]
	float beta;  // [rad]
	float ias;  // [m/s]
	float tas;  // [m/s]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_MHP_Products_t() {
		startByte = 0;
		alpha = 0.0;
		beta = 0.0;
		ias = 0.0;
		tas = 0.0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_MHP_Products_t;

typedef struct _CAN_MHP_Raw_t {
	uint8_t startByte;

	float differential_pressure[5];  // [Pa]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_MHP_Raw_t() {
		uint8_t _i;

		startByte = 0;

		for (_i = 0; _i < 5; ++_i)
			differential_pressure[_i] = 0.0;

		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_MHP_Raw_t;

typedef struct _CAN_Magnetometer_t {
	uint8_t startByte;

	float mx;  // [uT]
	float my;  // [uT]
	float mz;  // [uT]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_Magnetometer_t() {
		startByte = 0;
		mx = 0.0;
		my = 0.0;
		mz = 0.0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_Magnetometer_t;

typedef struct _CAN_Orientation_t {
	uint8_t startByte;

	float q[4];

	uint16_t chk;

#ifdef __cplusplus
	_CAN_Orientation_t() {
		uint8_t _i;

		startByte = 0;

		for (_i = 0; _i < 4; ++_i)
			q[_i] = 0.0;

		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_Orientation_t;

typedef struct _CAN_Pressure_t {
	uint8_t startByte;

	float pressureSta;  // [Pa]
	float pressureDyn;  // [Pa]
	float temp;  // [deg C]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_Pressure_t() {
		startByte = 0;
		pressureSta = 0.0;
		pressureDyn = 0.0;
		temp = 0.0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_Pressure_t;

typedef struct _CAN_Proximity_t {
	uint8_t startByte;

	float timestamp;
	float distance;
	float velocity;

	uint16_t chk;

#ifdef __cplusplus
	_CAN_Proximity_t() {
		startByte = 0;
		timestamp = 0.0;
		distance = 0.0;
		velocity = 0.0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_Proximity_t;

typedef struct _CAN_Trigger_t {
	uint8_t startByte;

	float timestamp;
	uint16_t id;  // [#] trigger (photo) number
	uint8_t channel;  // [#] payload channel

	uint16_t chk;

#ifdef __cplusplus
	_CAN_Trigger_t() {
		startByte = 0;
		timestamp = 0.0;
		id = 0;
		channel = 0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_Trigger_t;

typedef struct _CAN_Wind_t {
	uint8_t startByte;

	float u;  // [m/s]
	float v;  // [m/s]
	float w;  // [m/s]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_Wind_t() {
		startByte = 0;
		u = 0.0;
		v = 0.0;
		w = 0.0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_Wind_t;

/*--------[ SYSTEM ]--------*/

typedef struct _CAN_Supply_t {
	uint8_t startByte;

	float voltage;  // [V]
	float current;  // [A]
	float coulomb_count;  // [mAh]
	float temperature;  // [deg C]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_Supply_t() {
		startByte = 0;
		voltage = 0.0;
		current = 0.0;
		coulomb_count = 0.0;
		temperature = 0.0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_Supply_t;

/*--------[ Sensors ]--------*/

typedef enum {
	CAN_CAL_UNKNOWN,
	CAN_REQUESTED,
	CAN_SENT,
	CAN_CALIBRATED,
}  __attribute__ ((packed)) CAN_CalibrationState_t;

typedef enum {
	CAN_ACCELEROMETER,
	CAN_GYROSCOPE,
	CAN_MAGNETOMETER,
	CAN_DYNAMIC_PRESSURE,
	CAN_STATIC_PRESSURE,
	CAN_TEMPERATURE,
	CAN_HUMIDITY,
	CAN_AGL,
	CAN_GPS,
	CAN_SENSOR_PAYLOAD_1,
	CAN_SENSOR_PAYLOAD_2,
	CAN_SENSOR_PAYLOAD_3,
	CAN_SENSOR_PAYLOAD_4,
	CAN_SENSOR_PAYLOAD_5,
	CAN_UNKNOWN_SENSOR,
}  __attribute__ ((packed)) CAN_SensorType_t;

typedef struct _CAN_AxisMapping_t {
	uint8_t startByte;

	int8_t axis[3];

	uint16_t chk;

#ifdef __cplusplus
	_CAN_AxisMapping_t() {
		uint8_t _i;

		startByte = 0;

		for (_i = 0; _i < 3; ++_i)
			axis[_i] = 0;

		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_AxisMapping_t;

typedef struct _CAN_CalibrateSensor_t {
	uint8_t startByte;

	CAN_SensorType_t sensor;
	CAN_CalibrationState_t state;

	uint16_t chk;

#ifdef __cplusplus
	_CAN_CalibrateSensor_t() {
		startByte = 0;
		sensor = CAN_UNKNOWN_SENSOR;
		state = CAN_CAL_UNKNOWN;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_CalibrateSensor_t;

/*--------[ Status ]--------*/

typedef enum {
	CMD_HEARTBEAT,
	CMD_SET_STATE,
}  __attribute__ ((packed)) CAN_DeploymentTubeCommandID_t;

typedef enum {
	CLOSED,
	OPEN,
}  __attribute__ ((packed)) CAN_DeploymentTubeDoorStatus_t;

typedef enum {
	DEPLOY_TUBE_ERROR_NO_ERROR=0,
	DEPLOY_TUBE_ERROR_LOW_BATT=1,
	DEPLOY_TUBE_ERROR_HIGH_VOLTAGE=2,
	DEPLOY_TUBE_ERROR_NO_BATT=4,
	DEPLOY_TUBE_ERROR_HIGH_CURRENT=8,
	DEPLOY_TUBE_ERROR_HIGH_TEMP=16,
	DEPLOY_TUBE_ERROR_CHUTE_FLAP_OPEN=32,
	DEPLOY_TUBE_ERROR_CHUTE_FLAP_CLOSED=64,
	DEPLOY_TUBE_ERROR_NO_UA_COMMS=128,
}  __attribute__ ((packed)) CAN_DeploymentTubeErrors_t;

/*--------[ System ]--------*/

typedef struct _CAN_PowerOn_t {
	uint8_t startByte;

	uint16_t comms_rev;  // comms packet version
	uint32_t serial_num;  // serial number of vehicle

	uint16_t chk;

#ifdef __cplusplus
	_CAN_PowerOn_t() {
		startByte = 0;
		comms_rev = 0;
		serial_num = 0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_PowerOn_t;

typedef struct _CAN_GNSS_LLA_t {
	uint8_t startByte;

	double latitude;  // [deg]
	double longitude;  // [deg]
	float altitude;  // [m]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_GNSS_LLA_t() {
		startByte = 0;
		latitude = 0.0;
		longitude = 0.0;
		altitude = 0.0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_GNSS_LLA_t;

typedef struct _CAN_GNSS_RTCM_t {
	uint8_t startByte;

	uint8_t size;
	uint8_t payload[64];

	uint16_t chk;

#ifdef __cplusplus
	_CAN_GNSS_RTCM_t() {
		uint8_t _i;

		startByte = 0;
		size = 0;

		for (_i = 0; _i < 64; ++_i)
			payload[_i] = 0;

		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_GNSS_RTCM_t;

typedef struct _CAN_GNSS_SVIN_t {
	uint8_t startByte;

	uint32_t time_elapsed;  // [s] - time since start of survey in was requested
	uint32_t time_minimum;  // [s] - time required for survey in
	float accuracy;  // [s] - current accuracy of survey in
	float accuracy_minimum;  // [s] - minimum accuracy required for survey in
	uint8_t flags;  // see GCSRTKFlags_t

	uint16_t chk;

#ifdef __cplusplus
	_CAN_GNSS_SVIN_t() {
		startByte = 0;
		time_elapsed = 0;
		time_minimum = 0;
		accuracy = 0.0;
		accuracy_minimum = 0.0;
		flags = 0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_GNSS_SVIN_t;

typedef struct _CAN_GNSS_UTC_t {
	uint8_t startByte;

	uint8_t hours;  // [hrs]
	uint8_t minutes;  // [min]
	float seconds;  // [sec]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_GNSS_UTC_t() {
		startByte = 0;
		hours = 0;
		minutes = 0;
		seconds = 0.0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_GNSS_UTC_t;

typedef struct _CAN_GNSS_UTC_W_t {
	uint8_t startByte;

	uint16_t week;
	uint8_t hours;  // [hrs]
	uint8_t minutes;  // [min]
	float seconds;  // [sec]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_GNSS_UTC_W_t() {
		startByte = 0;
		week = 0;
		hours = 0;
		minutes = 0;
		seconds = 0.0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_GNSS_UTC_W_t;

/*--------[ ACTUATORS ]--------*/

typedef struct _CAN_Actuator_t {
	uint8_t startByte;

	uint16_t usec[16];  // [usec]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_Actuator_t() {
		uint8_t _i;

		startByte = 0;

		for (_i = 0; _i < 16; ++_i)
			usec[_i] = 0;

		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_Actuator_t;

/*--------[ INPUT ]--------*/

typedef struct _CAN_Receiver_t {
	uint8_t startByte;

	uint16_t usec[16];  // [usec]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_Receiver_t() {
		uint8_t _i;

		startByte = 0;

		for (_i = 0; _i < 16; ++_i)
			usec[_i] = 0;

		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_Receiver_t;

/*--------[ Telemetry ]--------*/

typedef struct _CAN_DeploymentTube_t {
	uint8_t startByte;

	CAN_DeploymentTubeState_t state;
	CAN_DeploymentTubeDoorStatus_t parachute_door;
	uint8_t batt_voltage;
	CAN_DeploymentTubeErrors_t error;

	uint16_t chk;

#ifdef __cplusplus
	_CAN_DeploymentTube_t() {
		startByte = 0;
		batt_voltage = 0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_DeploymentTube_t;

#ifdef __cplusplus
} /* namespace canpackets */

} /* namespace comms */
} /* namespace bst */
#endif

#endif /* _CANPACKETS_H_ */
