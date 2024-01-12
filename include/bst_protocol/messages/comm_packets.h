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

#ifndef _COMM_PACKETS_H_
#define _COMM_PACKETS_H_

#include <inttypes.h>

#ifdef __cplusplus
namespace bst {
namespace comms {
// namespace default {
#endif

/*--------[ Actuators ]--------*/

#define MAX_USEC 2200

#define MIN_USEC 800

#define NUM_ACTUATORS 16

#define NUM_MANUAL_CHANNELS 16

typedef enum {
	ACT_UNUSED,
	ACT_L_AILERON,
	ACT_L_ELEVATOR,
	ACT_L_THROTTLE,
	ACT_L_RUDDER,
	ACT_L_FLAP,
	ACT_L_RUDDERVATOR,
	ACT_L_ELEVON,
	ACT_L_GEAR,
	ACT_R_AILERON,
	ACT_R_ELEVATOR,
	ACT_R_THROTTLE,
	ACT_R_RUDDER,
	ACT_R_FLAP,
	ACT_R_RUDDERVATOR,
	ACT_R_ELEVON,
	ACT_R_GEAR,
	ACT_ROTOR,
	ACT_PAYLOAD_1,
	ACT_PAYLOAD_2,
	ACT_PAYLOAD_3,
	ACT_PAYLOAD_4,
	ACT_PAYLOAD_5,
	ACT_PAYLOAD_6,
	ACT_PAYLOAD_7,
	ACT_PAYLOAD_8,
	ACT_PAYLOAD_9,
	ACT_PAYLOAD_10,
	ACT_PAYLOAD_11,
	ACT_PAYLOAD_12,
	ACT_PAYLOAD_13,
	ACT_PAYLOAD_14,
	ACT_PAYLOAD_15,
	ACT_PAYLOAD_16,
	ACT_INVALID,
}  __attribute__ ((packed)) ActuatorFunction_t;

typedef enum {
	UNUSED_JS,
	JS_ROLL_RATE,
	JS_PITCH_RATE,
	JS_YAW_RATE,
	JS_X_VEL,
	JS_Y_VEL,
	JS_Z_VEL,
	JS_PAYLOAD_1,
	JS_PAYLOAD_2,
	INVALID_JOYSTICK,
}  __attribute__ ((packed)) JoystickFunction_t;

typedef enum {
	AILERON,
	ELEVATOR,
	THROTTLE,
	RUDDER,
	FLAPS,
	GEAR,
	PAYLOAD_1,
	PAYLOAD_2,
	PAYLOAD_3,
	PAYLOAD_4,
	PAYLOAD_5,
	PAYLOAD_6,
	PAYLOAD_7,
	PAYLOAD_8,
	PAYLOAD_9,
	PAYLOAD_10,
	PAYLOAD_11,
	PAYLOAD_12,
	PAYLOAD_13,
	PAYLOAD_14,
	PAYLOAD_15,
	PAYLOAD_16,
	INVALID_SURFACE,
}  __attribute__ ((packed)) SurfaceCommand_t;

typedef struct _ActuatorCalibration_t {
	uint8_t channel;
	ActuatorFunction_t type;
	uint16_t max_usec;
	uint16_t mid_usec;
	uint16_t min_usec;

#ifdef __cplusplus
	_ActuatorCalibration_t() {
		channel = 255;
		type = ACT_INVALID;
		max_usec = 1500;
		mid_usec = 1500;
		min_usec = 1500;
	}
#endif
} __attribute__ ((packed)) ActuatorCalibration_t;

typedef struct _Actuators_t {
	int16_t usec[16];

#ifdef __cplusplus
	_Actuators_t() {
		uint8_t _i;

		for (_i = 0; _i < 16; ++_i)
			usec[_i] = 0;
	}
#endif
} __attribute__ ((packed)) Actuators_t;

/*--------[ Comms ]--------*/

typedef enum {
	/* SENSORS */
	SENSORS_HUMIDITY=0,
	SENSORS_GPS=1,
	SENSORS_ACCELEROMETER=2,
	SENSORS_GYROSCOPE=3,
	SENSORS_MAGNETOMETER=4,
	SENSORS_IMU=5,
	SENSORS_DYNAMIC_PRESSURE=6,
	SENSORS_STATIC_PRESSURE=7,
	SENSORS_AIR_TEMPERATURE=8,
	SENSORS_AGL=9,
	SENSORS_CALIBRATE=10,
	SENSORS_BOARD_ORIENTATION=11,
	SENSORS_GNSS_ORIENTATION=12,
	SENSORS_MHP=13,
	SENSORS_GNSS_RTCM=14,
	SENSORS_MHP_SENSORS=15,
	SENSORS_MHP_9H_SENSORS=23,  // FIXME - TECHNICALLY IN STATE ADDR SPACE
	SENSORS_MHP_9H_TIMING=24,  // FIXME - TECHNICALLY IN STATE ADDR SPACE
	SENSORS_DYNP_CALIBRATION=25,  // FIXME - TECHNICALLY IN STATE ADDR SPACE
	SENSORS_GYRO_CALIBRATION=26,  // FIXME - TECHNICALLY IN STATE ADDR SPACE
	SENSORS_MAG_CALIBRATION=27,  // FIXME - TECHNICALLY IN STATE ADDR SPACE
	SENSORS_MAG_CURRENT_CAL=28,  // FIXME - TECHNICALLY IN STATE ADDR SPACE
	SENSORS_ADSB=29,  // FIXME - TECHNICALLY IN STATE ADDR SPACE
	SENSORS_MHP_GNSS=30,  // FIXME - TECHNICALLY IN STATE ADDR SPACE
	SENSORS_MHP_TIMING=31,  // FIXME - TECHNICALLY IN STATE ADDR SPACE

	/* STATE */
	STATE_STATE=16,  // ONLY USED INTERNALLY
	STATE_ESTIMATOR_PARAM=17,
	STATE_FLIGHT_CONTROLLER=18,  // ONLY USED INTERNALLY O

	/* CONTROL */
	/* CONTROL_VALUES                = 0x20 */
	CONTROL_COMMAND=33,
	CONTROL_PID=34,
	CONTROL_FLIGHT_PARAMS=35,
	CONTROL_FILTER_PARAMS=36,

	/* ACTUATORS */
	ACTUATORS_VALUES=48,
	ACTUATORS_CALIBRATION=49,
	ACTUATORS_ROTOR_PARAMS=50,
	ACTUATORS_MIXING_PARAMS=51,

	/* HANDSET */
	HANDSET_VALUES=52,
	HANDSET_CALIBRATION=53,

	/* INPUT */
	INPUT_HANDSET_VALUES=64,
	INPUT_HANDSET_SETUP=65,
	INPUT_JOYSTICK_VALUES=66,
	INPUT_JOYSTICK_SETUP=67,

	/* SYSTEM */
	SYSTEM_POWER_ON=80,
	SYSTEM_INITIALIZE=81,
	SYSTEM_HEALTH_AND_STATUS=82,  // FIXME - ONLY USED INTERNALLY
	SYSTEM_HARDWARE_ERROR=83,
	SYSTEM_REBOOT=95,

	/* TELEMETRY */
	TELEMETRY_HEARTBEAT=96,
	TELEMETRY_POSITION=97,
	TELEMETRY_ORIENTATION=98,
	TELEMETRY_PRESSURE=99,
	TELEMETRY_CONTROL=100,
	TELEMETRY_SYSTEM=101,
	TELEMETRY_GCS=102,
	TELEMETRY_GCS_LOCATION=103,
	TELEMETRY_PAYLOAD=104,
	TELEMETRY_GCS_SVIN=105,
	TELEMETRY_DEPLOYMENT_TUBE=121,  // FIXME - TECHNICALLY IN HIWL ADDR SPACE

	/* HWIL */
	HWIL_SENSORS=112,
	HWIL_ACTUATORS=113,
	HWIL_CAN=114,

	/* FLIGHT PLAN */
	FLIGHT_PLAN=128,
	FLIGHT_PLAN_MAP=129,
	FLIGHT_PLAN_WAYPOINT=130,
	LAST_MAPPING_WAYPOINT=131,
	DUBIN_PATH=132,

	/* VEHICLE CONFIGURATION */
	VEHICLE_PARAMS=144,
	VEHICLE_LIMITS=145,
	VEHICLE_LAUNCH_PARAMS=146,
	VEHICLE_LAND_PARAMS=147,

	/* MISSION */
	MISSION_CHECKLIST=160,
	MISSION_PARAMETERS=161,

	/* PAYLOAD */
	PAYLOAD_TRIGGER=224,
	PAYLOAD_PARAMS=225,
	PAYLOAD_NDVI=226,
	PAYLOAD_LDCR=227,
	PAYLOAD_CONTROL=228,
	PAYLOAD_CAMERA_TAG=229,
	PAYLOAD_STATUS=230,
	PAYLOAD_SERIAL=231,

	PAYLOAD_DATA_CHANNEL_0=232,
	PAYLOAD_DATA_CHANNEL_1=233,
	PAYLOAD_DATA_CHANNEL_2=234,
	PAYLOAD_DATA_CHANNEL_3=235,
	PAYLOAD_DATA_CHANNEL_4=236,
	PAYLOAD_DATA_CHANNEL_5=237,
	PAYLOAD_DATA_CHANNEL_6=238,
	PAYLOAD_DATA_CHANNEL_7=239,

	/* ERRORS */
	INVALID_PACKET=255,
}  __attribute__ ((packed)) PacketTypes_t;

/*--------[ Configuration ]--------*/

#define COMMS_VERSION 3170

#define MAX_ALTITUDE 20000

#define MAX_VEHICLES 5

/*--------[ Estimator ]--------*/

typedef struct _EstimatorParameters_t {
	float cf_acc_gain;
	float cf_mag_gain;
	float cf_press_gain;

	float altitude_filter_Ph;
	float altitude_filter_R;
	float altitude_filter_Q1;
	float altitude_filter_Q2;
	float altitude_filter_Q3;

	float ax_filter_alpha;
	float ax_filter_beta;

	float agl_filter_rate;
	float agl_filter_cutoff;

#ifdef __cplusplus
	_EstimatorParameters_t() {
		cf_acc_gain = 0.0;
		cf_mag_gain = 0.0;
		cf_press_gain = 0.0;
		altitude_filter_Ph = 0.0;
		altitude_filter_R = 0.0;
		altitude_filter_Q1 = 0.0;
		altitude_filter_Q2 = 0.0;
		altitude_filter_Q3 = 0.0;
		ax_filter_alpha = 0.0;
		ax_filter_beta = 0.0;
		agl_filter_rate = 0.0;
		agl_filter_cutoff = 0.0;
	}
#endif
} __attribute__ ((packed)) EstimatorParameters_t;

typedef struct _State_t {
	float system_time;  // [sec]
	float q[4];
	float altitude;  // [m]
	float ias;
	float ias_dot;
	float h;
	float h_dot;
	float turnrate;
	float accel_y;
	double latitude;
	double longitude;
	float vx;
	float vy;
	float agl;  // [m]
	float tas;  // [m/s]
	float wind[3];  // [m/s]

#ifdef __cplusplus
	_State_t() {
		uint8_t _i;

		system_time = 0.0;

		for (_i = 0; _i < 4; ++_i)
			q[_i] = 0.0;

		altitude = 0.0;
		ias = 0.0;
		ias_dot = 0.0;
		h = 0.0;
		h_dot = 0.0;
		turnrate = 0.0;
		accel_y = 0.0;
		latitude = 0.0;
		longitude = 0.0;
		vx = 0.0;
		vy = 0.0;
		agl = 0.0;
		tas = 0.0;

		for (_i = 0; _i < 3; ++_i)
			wind[_i] = 0.0;
	}
#endif
} __attribute__ ((packed)) State_t;

/*--------[ FlightPlan ]--------*/

#define ERROR_WAYPOINT 118

#define INVALID_WAYPOINT 255

#define LANDING_WAYPOINT_1 101

#define LANDING_WAYPOINT_2 107

#define LAUNCH_WAYPOINT 113

#define LOOK_AT_POINT 120

#define LOST_COMM_WAYPOINT 119

#define MAX_LOOK_AT_POINTS 10

#define MAX_USER_WAYPOINTS 100

#define MAX_WAYPOINTS 130

#define VIRTUAL_WAYPOINT 114

#define WAYPOINT_MAP_SIZE 17

typedef enum {
	RSR,
	LSL,
	LSR,
	RSL,
	LRL,
	RLR,
	ORBIT,
	HELIX,
	NEXT,
	GOTO,
	GOTO_MR,
}  __attribute__ ((packed)) DubinsPathType_t;

typedef enum {
	NONE,
	ADD,
	DELETE,
	FINISH,
}  __attribute__ ((packed)) FPMapMode_t;

typedef enum {
	ACTION_NONE=0,
	BASE=1,
	RELATIVE=2,
	AGL_ALT=4,
	AYL_ALT=8,
	LANDING=16,
	TAKEOFF=32,
	FLY_OVER=64,
	MAP_TO=128,
	MAP_FROM=256,
	END_OF_RUNWAY=512,
}  __attribute__ ((packed)) WaypointAction_t;

typedef struct _Waypoint_t {
	uint8_t num;
	uint8_t next;
	double latitude;  // [deg]
	double longitude;  // [deg]
	float altitude;  // [m]
	float radius;  // [m]
	uint16_t action;  // bitwise field of actions

#ifdef __cplusplus
	_Waypoint_t() {
		num = 255;
		next = 255;
		latitude = 0.0;
		longitude = 0.0;
		altitude = 0.0;
		radius = 0.0;
		action = 0;
	}
#endif
} __attribute__ ((packed)) Waypoint_t;

typedef struct _DubinsPath_t {
	float xc0_x;
	float xc0_y;
	float xc1_x;
	float xc1_y;
	float xc2_x;
	float xc2_y;
	float t0_i;
	float t0_f;
	float t1_i;
	float t1_f;
	float t2_i;
	float t2_f;
	float a;
	float b;
	float c0;
	float c1;
	float c2;
	float x0;
	float y0;
	float z00;
	float z01;
	float z02;
	float big_r;
	double origin_lat;
	double origin_lon;
	DubinsPathType_t path_type;

#ifdef __cplusplus
	_DubinsPath_t() {
		xc0_x = 0.0;
		xc0_y = 0.0;
		xc1_x = 0.0;
		xc1_y = 0.0;
		xc2_x = 0.0;
		xc2_y = 0.0;
		t0_i = 0.0;
		t0_f = 0.0;
		t1_i = 0.0;
		t1_f = 0.0;
		t2_i = 0.0;
		t2_f = 0.0;
		a = 0.0;
		b = 0.0;
		c0 = 0.0;
		c1 = 0.0;
		c2 = 0.0;
		x0 = 0.0;
		y0 = 0.0;
		z00 = 0.0;
		z01 = 0.0;
		z02 = 0.0;
		big_r = 0.0;
		origin_lat = 0.0;
		origin_lon = 0.0;
		path_type = GOTO;
	}
#endif
} __attribute__ ((packed)) DubinsPath_t;

typedef struct _FlightPlanMap_t {
	FPMapMode_t mode;
	uint8_t map[17];

#ifdef __cplusplus
	_FlightPlanMap_t() {
		uint8_t _i;

		mode = NONE;

		for (_i = 0; _i < 17; ++_i)
			map[_i] = 0;
	}
#endif
} __attribute__ ((packed)) FlightPlanMap_t;

/*--------[ Mission ]--------*/

typedef enum {
	MISSION_LIMITS_INITIALIZED=1,
	MISSION_LOST_COMMS_INITIALIZED=2,
	MISSION_START_WP_INITIALIZED=4,
}  __attribute__ ((packed)) MissionInitialization_t;

/*--------[ Sensors ]--------*/

#define FLAGS_BARO_VALID 256

#define FLAGS_SIMULATED 64

#define FLAGS_SOURCE_UAT 32767

#define FLAGS_VALID_ALTITUDE 2

#define FLAGS_VALID_CALLSIGN 16

#define FLAGS_VALID_COORDS 1

#define FLAGS_VALID_HEADING 4

#define FLAGS_VALID_SQUAWK 32

#define FLAGS_VALID_VELOCITY 8

#define FLAGS_VERTICAL_VELOCITY_VALID 128

typedef enum {
	/* Altitude reported from a Baro source using QNH reference */
	ALTITUDE_TYPE_PRESSURE_QNH,
	/* Altitude reported from a GNSS source */
	ALTITUDE_TYPE_GEOMETRIC,
}  __attribute__ ((packed)) ADSB_Altitude_t;

typedef enum {
	EMITTER_TYPE_NO_INFO,
	EMITTER_TYPE_LIGHT,
	EMITTER_TYPE_SMALL,
	EMITTER_TYPE_LARGE,
	EMITTER_TYPE_HIGH_VORTEX_LARGE,
	EMITTER_TYPE_HEAVY,
	EMITTER_TYPE_HIGHLY_MANUV,
	EMITTER_TYPE_ROTOCRAFT,
	EMITTER_TYPE_UNASSIGNED,
	EMITTER_TYPE_GLIDER,
	EMITTER_TYPE_LIGHTER_AIR,
	EMITTER_TYPE_PARACHUTE,
	EMITTER_TYPE_ULTRA_LIGHT,
	EMITTER_TYPE_UNASSIGNED2,
	EMITTER_TYPE_UAV,
	EMITTER_TYPE_SPACE,
	EMITTER_TYPE_UNASSGINED3,
	EMITTER_TYPE_EMERGENCY_SURFACE,
	EMITTER_TYPE_SERVICE_SURFACE,
	EMITTER_TYPE_POINT_OBSTACLE,
}  __attribute__ ((packed)) ADSB_Emitter_t;

typedef enum {
	CAL_UNKNOWN,
	REQUESTED,
	SENT,
	CALIBRATED,
}  __attribute__ ((packed)) CalibrationState_t;

typedef enum {
	NO_FIX,
	DEAD_RECKONING_ONLY,
	FIX_2D,
	FIX_3D,
	GNSS_DEAD_RECKONING,
	TIME_ONLY,
	DGPS,
	RTK,
}  __attribute__ ((packed)) GPSFixType_t;

typedef enum {
	ACCELEROMETER,
	GYROSCOPE,
	MAGNETOMETER,
	DYNAMIC_PRESSURE,
	STATIC_PRESSURE,
	TEMPERATURE,
	HUMIDITY,
	AGL,
	GPS,
	SENSOR_PAYLOAD_1,
	SENSOR_PAYLOAD_2,
	SENSOR_PAYLOAD_3,
	SENSOR_PAYLOAD_4,
	SENSOR_PAYLOAD_5,
	UNKNOWN_SENSOR,
}  __attribute__ ((packed)) SensorType_t;

typedef struct _AxisMapping_t {
	int8_t axis[3];

#ifdef __cplusplus
	_AxisMapping_t() {
		uint8_t _i;

		for (_i = 0; _i < 3; ++_i)
			axis[_i] = 0;
	}
#endif
} __attribute__ ((packed)) AxisMapping_t;

typedef struct _MHP_t {
	float system_time;  // [s]
	float alpha;  // [rad]
	float beta;  // [rad]
	float q;  // [m/s]
	float ias;  // [m/s]
	float tas;  // [m/s]
	float wind[3];  // [m/s]
	float quaternion[4];

#ifdef __cplusplus
	_MHP_t() {
		uint8_t _i;

		system_time = 0.0;
		alpha = 0.0;
		beta = 0.0;
		q = 0.0;
		ias = 0.0;
		tas = 0.0;

		for (_i = 0; _i < 3; ++_i)
			wind[_i] = 0.0;

		for (_i = 0; _i < 4; ++_i)
			quaternion[_i] = 0.0;
	}
#endif
} __attribute__ ((packed)) MHP_t;

typedef struct _MHP9HSensors_t {
	float system_time;  // [s]
	uint8_t error_code;
	float static_pressure;  // [Pa]
	float dynamic_pressure[9];  // [Pa]
	float air_temperature;  // [deg C]
	float humidity;  // [%]
	float gyroscope[3];  // [rad/s]
	float accelerometer[3];  // [g]

#ifdef __cplusplus
	_MHP9HSensors_t() {
		uint8_t _i;

		system_time = 0.0;
		error_code = 0;
		static_pressure = 0.0;

		for (_i = 0; _i < 9; ++_i)
			dynamic_pressure[_i] = 0.0;

		air_temperature = 0.0;
		humidity = 0.0;

		for (_i = 0; _i < 3; ++_i)
			gyroscope[_i] = 0.0;

		for (_i = 0; _i < 3; ++_i)
			accelerometer[_i] = 0.0;
	}
#endif
} __attribute__ ((packed)) MHP9HSensors_t;

typedef struct _MHP9HTiming_t {
	float system_time;  // [s]
	float static_pressure_time;  // [s]
	float dynamic_pressure_time[9];  // [s]
	float air_temperature_time;  // [s]
	float humidity_time;  // [s]
	float imu_time;  // [s]
	float magnetometer_time;  // [s]
	float gps_time;  // [s]

#ifdef __cplusplus
	_MHP9HTiming_t() {
		uint8_t _i;

		system_time = 0.0;
		static_pressure_time = 0.0;

		for (_i = 0; _i < 9; ++_i)
			dynamic_pressure_time[_i] = 0.0;

		air_temperature_time = 0.0;
		humidity_time = 0.0;
		imu_time = 0.0;
		magnetometer_time = 0.0;
		gps_time = 0.0;
	}
#endif
} __attribute__ ((packed)) MHP9HTiming_t;

typedef struct _MHPSensors_t {
	float system_time;  // [s]
	uint8_t error_code;
	float static_pressure;  // [Pa]
	float dynamic_pressure[5];  // [Pa]
	float air_temperature;  // [deg C]
	float humidity;  // [%]
	float gyroscope[3];  // [rad/s]
	float accelerometer[3];  // [g]

#ifdef __cplusplus
	_MHPSensors_t() {
		uint8_t _i;

		system_time = 0.0;
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
	}
#endif
} __attribute__ ((packed)) MHPSensors_t;

typedef struct _MHPSensorsGNSS_t {
	float system_time;  // [s]
	float magnetometer[3];  // [uT]
	uint16_t week;
	uint8_t hour;
	uint8_t minute;
	float seconds;
	double latitude;  // [deg]
	double longitude;  // [deg]
	float altitude;  // [m]
	float velocity[3];  // [m/s]
	float pdop;

#ifdef __cplusplus
	_MHPSensorsGNSS_t() {
		uint8_t _i;

		system_time = 0.0;

		for (_i = 0; _i < 3; ++_i)
			magnetometer[_i] = 0.0;

		week = 0;
		hour = 0;
		minute = 0;
		seconds = 0.0;
		latitude = 0.0;
		longitude = 0.0;
		altitude = 0.0;

		for (_i = 0; _i < 3; ++_i)
			velocity[_i] = 0.0;

		pdop = 0.0;
	}
#endif
} __attribute__ ((packed)) MHPSensorsGNSS_t;

typedef struct _MHPTiming_t {
	float system_time;  // [s]
	float static_pressure_time;  // [s]
	float dynamic_pressure_time[5];  // [s]
	float air_temperature_time;  // [s]
	float humidity_time;  // [s]
	float imu_time;  // [s]
	float magnetometer_time;  // [s]
	float gps_time;  // [s]

#ifdef __cplusplus
	_MHPTiming_t() {
		uint8_t _i;

		system_time = 0.0;
		static_pressure_time = 0.0;

		for (_i = 0; _i < 5; ++_i)
			dynamic_pressure_time[_i] = 0.0;

		air_temperature_time = 0.0;
		humidity_time = 0.0;
		imu_time = 0.0;
		magnetometer_time = 0.0;
		gps_time = 0.0;
	}
#endif
} __attribute__ ((packed)) MHPTiming_t;

typedef struct _Pressure_t {
	float system_time;
	float pressure;
	float temperature;

#ifdef __cplusplus
	_Pressure_t() {
		system_time = 0.0;
		pressure = 0.0;
		temperature = 0.0;
	}
#endif
} __attribute__ ((packed)) Pressure_t;

typedef struct _RTCM_t {
	uint8_t size;
	uint8_t payload[64];

#ifdef __cplusplus
	_RTCM_t() {
		uint8_t _i;

		size = 0;

		for (_i = 0; _i < 64; ++_i)
			payload[_i] = 0;
	}
#endif
} __attribute__ ((packed)) RTCM_t;

typedef struct _SensorOffsets_t {
	uint8_t status;
	float gyroscope_x;
	float gyroscope_y;
	float gyroscope_z;
	float magnetometer_x;
	float magnetometer_y;
	float magnetometer_z;
	float dynamic_pressure;
	float temperature;

#ifdef __cplusplus
	_SensorOffsets_t() {
		status = 0;
		gyroscope_x = 0.0;
		gyroscope_y = 0.0;
		gyroscope_z = 0.0;
		magnetometer_x = 0.0;
		magnetometer_y = 0.0;
		magnetometer_z = 0.0;
		dynamic_pressure = 0.0;
		temperature = 0.0;
	}
#endif
} __attribute__ ((packed)) SensorOffsets_t;

typedef struct _SingleAxisSensorCalibration_t {
	float b;  // Offset Bias
	float m;  // Scale Factor

#ifdef __cplusplus
	_SingleAxisSensorCalibration_t() {
		b = 0.0;
		m = 0.0;
	}
#endif
} __attribute__ ((packed)) SingleAxisSensorCalibration_t;

typedef struct _SingleValue_t {
	float value;

#ifdef __cplusplus
	_SingleValue_t() {
		value = 0.0;
	}
#endif
} __attribute__ ((packed)) SingleValue_t;

typedef struct _SingleValueSensor_t {
	float system_time;
	float value;

#ifdef __cplusplus
	_SingleValueSensor_t() {
		system_time = 0.0;
		value = 0.0;
	}
#endif
} __attribute__ ((packed)) SingleValueSensor_t;

typedef struct _ThreeAxisSensor_t {
	float system_time;
	float x;
	float y;
	float z;

#ifdef __cplusplus
	_ThreeAxisSensor_t() {
		system_time = 0.0;
		x = 0.0;
		y = 0.0;
		z = 0.0;
	}
#endif
} __attribute__ ((packed)) ThreeAxisSensor_t;

typedef struct _ThreeAxisSensorCalibration_t {
	float b[3];  // Offset Bias
	float m[9];  // Homography (transformation) Matrix

#ifdef __cplusplus
	_ThreeAxisSensorCalibration_t() {
		uint8_t _i;

		for (_i = 0; _i < 3; ++_i)
			b[_i] = 0.0;

		for (_i = 0; _i < 9; ++_i)
			m[_i] = 0.0;
	}
#endif
} __attribute__ ((packed)) ThreeAxisSensorCalibration_t;

typedef struct _ADSB_t {
	float system_time;
	uint32_t icao_address;
	double latitude;
	double longitude;
	ADSB_Altitude_t altitude_type;
	float altitude;
	float heading;
	float horizontal_velocity;
	float vertical_velocity;
	char callsign[9];
	ADSB_Emitter_t emitter_type;
	uint8_t tslc;
	uint16_t flags;
	uint16_t squawk;

#ifdef __cplusplus
	_ADSB_t() {
		uint8_t _i;

		system_time = 0.0;
		icao_address = 0;
		latitude = 0.0;
		longitude = 0.0;
		altitude = 0.0;
		heading = 0.0;
		horizontal_velocity = 0.0;
		vertical_velocity = 0.0;

		for (_i = 0; _i < 9; ++_i)
			callsign[_i] = 0;

		tslc = 0;
		flags = 0;
		squawk = 0;
	}
#endif
} __attribute__ ((packed)) ADSB_t;

typedef struct _CalibrateSensor_t {
	SensorType_t sensor;
	CalibrationState_t state;

#ifdef __cplusplus
	_CalibrateSensor_t() {
		sensor = UNKNOWN_SENSOR;
		state = CAL_UNKNOWN;
	}
#endif
} __attribute__ ((packed)) CalibrateSensor_t;

typedef struct _GPS_t {
	float system_time;
	uint16_t week;
	uint8_t hour;
	uint8_t minute;
	float seconds;
	double latitude;
	double longitude;
	float altitude;
	float speed;
	float course;
	uint8_t satellites;
	float pdop;
	float last_fix;
	GPSFixType_t fix_type;
	ThreeAxisSensor_t velocity;

#ifdef __cplusplus
	_GPS_t() {
		system_time = 0.0;
		week = 0;
		hour = 0;
		minute = 0;
		seconds = 0.0;
		latitude = 0.0;
		longitude = 0.0;
		altitude = 0.0;
		speed = 0.0;
		course = 0.0;
		satellites = 0;
		pdop = 0.0;
		last_fix = -1.0;
		fix_type = NO_FIX;
	}
#endif
} __attribute__ ((packed)) GPS_t;

typedef struct _IMU_t {
	ThreeAxisSensor_t accelerometer;
	ThreeAxisSensor_t gyroscope;
	ThreeAxisSensor_t magnetometer;
	SingleValueSensor_t temperature;

#ifdef __cplusplus
	_IMU_t() {

	}
#endif
} __attribute__ ((packed)) IMU_t;

typedef struct _ThreeAxisFirstOrderCorrection_t {
	SensorType_t sensor;
	float x[2];
	float y[2];
	float z[2];

#ifdef __cplusplus
	_ThreeAxisFirstOrderCorrection_t() {
		uint8_t _i;

		sensor = UNKNOWN_SENSOR;

		for (_i = 0; _i < 2; ++_i)
			x[_i] = 0.0;

		for (_i = 0; _i < 2; ++_i)
			y[_i] = 0.0;

		for (_i = 0; _i < 2; ++_i)
			z[_i] = 0.0;
	}
#endif
} __attribute__ ((packed)) ThreeAxisFirstOrderCorrection_t;

typedef struct _Sensors_t {
	IMU_t imu;
	GPS_t gps;
	Pressure_t dynamic_pressure;
	Pressure_t static_pressure;
	SingleValueSensor_t air_temperature;
	SingleValueSensor_t humidity;
	SingleValueSensor_t agl;

#ifdef __cplusplus
	_Sensors_t() {

	}
#endif
} __attribute__ ((packed)) Sensors_t;

/*--------[ Status ]--------*/

typedef enum {
	HW_ERROR_WATCHDOG_RESET,
	HW_ERROR_I2C_BUS_LOCK,
	HW_ERROR_CAN_BUS_LOCK,
	HW_ERROR_HARD_FAULT,
	HW_ERROR_MEM_MANAGE,
	HW_ERROR_BUS_FAULT,
	HW_ERROR_USAGE_FAULT,
}  __attribute__ ((packed)) HardwareErrors_t;

typedef enum {
	IRQ_INVALID,
	IRQ_SYSTICK,
	IRQ_TIMER,
	IRQ_I2C1,
	IRQ_I2C2,
	IRQ_CAN1_RX,
	IRQ_CAN1_TX,
	IRQ_CAN2_RX,
	IRQ_CAN2_TX,
	IRQ_USART1,
	IRQ_USART2,
	IRQ_UART4,
	IRQ_SDIO,
	IRQ_COMM_RX,
	IRQ_COMM_TX,
	IRQ_DMA2_STREAM5,
	IRQ_DMA2_STREAM7,
	IRQ_DMA1_STREAM2,
	IRQ_DMA1_STREAM5,
	IRQ_DMA1_STREAM6,
	IRQ_DMA1_STREAM1,
	IRQ_DMA1_STREAM4,
	IRQ_DMA1_STREAM3,
	IRQ_DMA2_STREAM3,
	IRQ_DMA2_STREAM6,
}  __attribute__ ((packed)) InterruptTags_t;

typedef enum {
	ERROR_NO_ERROR=0,
	ERROR_LOW_BATT=1,
	ERROR_HIGH_VOLTAGE=2,
	ERROR_NO_BATT=4,
	ERROR_NO_GPS=8,
	ERROR_NO_RADIO=16,
	ERROR_HIGH_CURRENT=32,
	ERROR_HIGH_TEMP=64,
	ERROR_ENGINE_OUT=128,
	ERROR_NAVIGATION_ERROR=256,
	ERROR_CONTROLLER_ERROR=512,
	ERROR_BAD_IAS=1024,
	ERROR_FLYING_NO_PREFLIGHT=2048,
	ERROR_NO_PAYLOAD_ACTUATOR=4096,
	ERROR_BAD_FLIGHTPLAN=8192,
	ERROR_NO_SD_CARD=16384,
	ERROR_SD_CARD_ERROR=32768,
	ERROR_GEOFENCE=65536,
	ERROR_BAD_GPS=131072,
	ERROR_NO_LASER=262144,
	ERROR_NO_STATIC_PRESS=524288,
	ERROR_NO_MAG=1048576,
	ERROR_COMM_LIMIT=2097152,
	ERROR_BAD_COMMS=4194304,
	ERROR_BAD_HANDSET=8388608,
	ERROR_BATT_LIMIT=16777216,
	ERROR_CRITICAL_BATT=33554432,
	ERROR_HW_FAULT=67108864,
	ERROR_BAD_PROPULSION=134217728,
	ERROR_ICING=268435456,
	ERROR_BAD_LAUNCH=536870912,
	ERROR_RESET_IN_FLIGHT=1073741824,
	/* ERROR_WIND_LIMIT          = 0x00000000 */
	/* ERROR_TEMPERATURE_LIMIT   = 0x00000000 */
	/* ERROR_DA_LIMIT            = 0x00000000 */
}  __attribute__ ((packed)) SystemErrors_t;

typedef enum {
	TASK_INVALID,
	TASK_SENSORS,
	TASK_ESTIMATOR,
	TASK_CONTROLLER,
	TASK_ACTUATORS,
	TASK_LOGGING,
	TASK_SAVE_PARAM,
	TASK_INPUT,
	TASK_COMM_RX,
	TASK_COMM_TX,
	TASK_SYSTEM,
	TASK_EEPROM,
	TASK_SDCARD,
}  __attribute__ ((packed)) TaskTags_t;

/*--------[ System ]--------*/

typedef enum {
	HW_UNKNOWN,
	HW_LITE,
	HW_TEMPEST,
	HW_PRO,
	HW_SWIL,
	HW_XPLANE,
}  __attribute__ ((packed)) HardwareModel_t;

typedef enum {
	RADIO_SOCKET,
	RADIO_SERIAL,
	RADIO_P900,
	RADIO_XBEE,
	RADIO_XTEND,
	RADIO_INVALID,
}  __attribute__ ((packed)) RadioType_t;

typedef enum {
	INITALIZE,
	START_UP,
	PREFLIGHT,
	FLIGHT,
	HWIL,
}  __attribute__ ((packed)) SystemMode_t;

typedef enum {
	VEHICLE_UNKNOWN,
	FIXED_WING,
	MULTI_COPTER,
	GCS,
	PAYLOAD_NODE,
	TAIL_SITTER,
	VTOL,
}  __attribute__ ((packed)) VehicleType_t;

typedef struct _HardwareError_t {
	uint8_t error_code;  // error code
	uint32_t r0;  // register 0
	uint32_t r1;  // register 1
	uint32_t r2;  // register 2
	uint32_t r3;  // register 3
	uint32_t r12;  // register 12
	uint32_t lr;  // subroutine call address
	uint32_t pc;  // program counter
	uint32_t psr;  // program status register
	uint8_t task;  // task indicator
	uint8_t irq;  // interrupt indicator

#ifdef __cplusplus
	_HardwareError_t() {
		error_code = 0;
		r0 = 0;
		r1 = 0;
		r2 = 0;
		r3 = 0;
		r12 = 0;
		lr = 0;
		pc = 0;
		psr = 0;
		task = 0;
		irq = 0;
	}
#endif
} __attribute__ ((packed)) HardwareError_t;

typedef struct _PowerOn_t {
	uint16_t comms_rev;  // comms packet version
	uint32_t serial_num;  // serial number of vehicle

#ifdef __cplusplus
	_PowerOn_t() {
		comms_rev = 0;
		serial_num = 0;
	}
#endif
} __attribute__ ((packed)) PowerOn_t;

typedef struct _SystemStatus_t {
	float batt_voltage;  // battery voltage
	float batt_current;  // battery current
	float batt_percent;  // battery capacity
	float batt_coulomb_count;  // battery consumption
	float flight_time;  // time aloft
	int8_t rssi;  // from ground station
	uint8_t lost_comm;  // set if UA lost comm with GCS
	uint8_t lost_gps;  // set if UA lost GPS
	uint32_t error_code;  // set if system has any errors

#ifdef __cplusplus
	_SystemStatus_t() {
		batt_voltage = 0.0;
		batt_current = 0.0;
		batt_percent = 0.0;
		batt_coulomb_count = 0.0;
		flight_time = 0.0;
		rssi = 0;
		lost_comm = 1;
		lost_gps = 1;
		error_code = 0;
	}
#endif
} __attribute__ ((packed)) SystemStatus_t;

typedef struct _SystemInitialize_t {
	VehicleType_t vehicle_type;  // type of vehicle
	HardwareModel_t model;  // type of hardware
	uint16_t sw_rev;  // Software version
	uint16_t hw_rev;  // hardware version
	uint32_t svn_rev;  // code version number
	uint16_t comms_rev;  // comms packet version
	uint32_t serial_num;  // serial number of vehicle
	uint8_t vehicle_initialized;  // bitwise field of init systems
	uint8_t num_inputs;  // number of physical input channels from handset
	uint8_t num_actuators;  // number of physical output channels
	char name[16];  // name of vehicle
	/* -- */
	/* We have a total of 64 bytes to us before we need to adust eeprom layout */
	/* for futureproofing, we are adding 7 bytes to get us a 40 byte packet. we */
	/* currently have 31 more bytes (including the 7) to extend the packet */
	uint8_t unused[5];  // place holder for future parameters

#ifdef __cplusplus
	_SystemInitialize_t() {
		uint8_t _i;

		vehicle_type = VEHICLE_UNKNOWN;
		model = HW_UNKNOWN;
		sw_rev = 0;
		hw_rev = 0;
		svn_rev = 0;
		comms_rev = 0;
		serial_num = 0;
		vehicle_initialized = 0;
		num_inputs = 0;
		num_actuators = 0;

		for (_i = 0; _i < 16; ++_i)
			name[_i] = 0;

		for (_i = 0; _i < 5; ++_i)
			unused[_i] = 0;
	}
#endif
} __attribute__ ((packed)) SystemInitialize_t;

/*--------[ Vehicle ]--------*/

typedef enum {
	BATT_CHEM_LIPO,
	BATT_CHEM_LIHV,
	BATT_CHEM_LIION,
	BATT_CHEM_INVALID,
}  __attribute__ ((packed)) BatteryChemistry_t;

/*--------[ Input ]--------*/

typedef enum {
	HS_FUNC_UNUSED,
	HS_FUNC_SET_AUTO,  // follow waypoints
	HS_FUNC_SET_POS,  // follow joystick
	HS_FUNC_SET_PILOT,  // follow pilot (manual rc) command
	HS_FUNC_L_AILERON,
	HS_FUNC_L_ELEVATOR,
	HS_FUNC_L_THROTTLE,
	HS_FUNC_L_RUDDER,
	HS_FUNC_L_FLAP,
	HS_FUNC_L_GEAR,
	HS_FUNC_R_AILERON,
	HS_FUNC_R_ELEVATOR,
	HS_FUNC_R_THROTTLE,
	HS_FUNC_R_RUDDER,
	HS_FUNC_R_FLAP,
	HS_FUNC_R_GEAR,

	HS_FUNC_PAYLOAD_1,
	HS_FUNC_PAYLOAD_2,
	HS_FUNC_PAYLOAD_3,
	HS_FUNC_PAYLOAD_4,
	HS_FUNC_PAYLOAD_5,
	HS_FUNC_PAYLOAD_6,
	HS_FUNC_PAYLOAD_7,
	HS_FUNC_PAYLOAD_8,
	HS_FUNC_PAYLOAD_9,
	HS_FUNC_PAYLOAD_10,
	HS_FUNC_PAYLOAD_11,
	HS_FUNC_PAYLOAD_12,
	HS_FUNC_PAYLOAD_13,
	HS_FUNC_PAYLOAD_14,
	HS_FUNC_PAYLOAD_15,
	HS_FUNC_PAYLOAD_16,

	HS_FUNC_ROLL,
	HS_FUNC_PITCH,

	HS_FUNC_ROLL_RATE,
	HS_FUNC_PITCH_RATE,
	HS_FUNC_YAW_RATE,

	HS_FUNC_X_VEL,
	HS_FUNC_Y_VEL,
	HS_FUNC_Z_VEL,

	HS_FUNC_THRUST,

	HS_FUNC_SET_MOTORS,  // arm / disarm motors

	/* pilot mode */
	HS_FUNC_SET_ACRO,  // angular rates, and v-rate
	HS_FUNC_SET_ANGLE,  // angles, and v-rate
	HS_FUNC_SET_COORD,  // coordinated turn

	/* position mode */
	HS_FUNC_SET_BODY,  // body frame velocity input
	HS_FUNC_SET_WORLD,  // world frame velocity input
	HS_FUNC_SET_BASE,  // polar frame velocity inputs

	HS_FUNC_INVALID,
}  __attribute__ ((packed)) HandsetFunction_t;

typedef enum {
	HS_TYPE_LINEAR,
	HS_TYPE_TOGGLE,
	HS_TYPE_3WAY,
	HS_TYPE_INVALID,
}  __attribute__ ((packed)) HandsetType_t;

typedef struct _HandsetValues_t {
	uint16_t usec[16];

#ifdef __cplusplus
	_HandsetValues_t() {
		uint8_t _i;

		for (_i = 0; _i < 16; ++_i)
			usec[_i] = 0;
	}
#endif
} __attribute__ ((packed)) HandsetValues_t;

/*--------[ Controller ]--------*/

#define JOYSTICK_BUTTONS_BYTES 2

#define MAX_JOYSTICK_BUTTONS 16

#define MAX_JOYSTICK_CHANNELS 4

typedef enum {
	ALT_MODE_TAKE_OFF,
	ALT_MODE_HOLD,  // use altitude controller to track commanded alt
	ALT_MODE_RATE,  // throttle is climb or sink rate
	ALT_MODE_AUTO,  // use altitude controller to track waypoint alt
	ALT_MODE_LAND,
	ALT_MODE_EXTERNAL,
	ALT_MODE_DIRECT,  // throttle is passed directly as thrust command
	ALT_MODE_INVALID,
}  __attribute__ ((packed)) AltitudeControlMode_t;

typedef enum {
	AP_MODE_INIT,
	AP_MODE_AUTOMATIC,
	AP_MODE_MANUAL,
	AP_MODE_JOYSTICK,
	AP_MODE_INVALID_MODE,
}  __attribute__ ((packed)) AutopilotMode_t;

typedef enum {
	/* NOTE - you must check the numbers in the CommandID values */
	/* contained in the children folders if you modify or change these numbers */
	/*  */
	/* States */
	CMD_AUTOPILOT_MODE=0,
	CMD_FLIGHT_MODE=1,
	CMD_LANDING_MODE=2,

	/* Controller Modes */
	CMD_ALT_MODE=3,
	CMD_LAT_MODE=4,
	CMD_NAV_MODE=5,

	/* System Commands */
	CMD_ENGINE_KILL=6,
	CMD_FLIGHT_TERMINATE=7,
	CMD_ABORT=8,

	/* Navigation */
	CMD_WAYPOINT=9,
	CMD_WPT_ALT=16,
	CMD_TURN_RATE=10,

	/* Flight Actions */
	CMD_LAUNCH=11,
	CMD_LAND=12,

	/* Attitude */
	CMD_ROLL=13,
	CMD_PITCH=14,
	CMD_YAW=15,

	CMD_ROLL_RATE=17,
	CMD_PITCH_RATE=18,
	CMD_YAW_RATE=19,

	/* Altitude */
	CMD_ALTITUDE=20,
	CMD_VRATE=21,

	/* Logging Commands */
	CMD_DOWNLOAD_LOG=22,

	/* Payload Commands */
	CMD_TRIGGER_PAYLOAD=23,

	CMD_INVALID=37,
}  __attribute__ ((packed)) CommandID_t;

typedef enum {
	FLIGHT_MODE_INIT,
	FLIGHT_MODE_PREFLIGHT,
	FLIGHT_MODE_CALIBRATE,
	FLIGHT_MODE_LAUNCH,
	FLIGHT_MODE_CLIMBOUT,
	FLIGHT_MODE_TRANSITION_TO_FORWARD,
	FLIGHT_MODE_FLYING,
	FLIGHT_MODE_TRANSITION_TO_HOVER,
	FLIGHT_MODE_LANDING,
	FLIGHT_MODE_LANDED,
	FLIGHT_MODE_POSTFLIGHT,
	FLIGHT_MODE_TERMINATE,
	FLIGHT_MODE_INVALID_MODE,
}  __attribute__ ((packed)) FlightMode_t;

typedef enum {
	LAT_MODE_WINGS_LEVEL,  // this also has heading (yaw) hold for ground handling
	LAT_MODE_COURSE_HOLD,  // this is a course, not a heading, hold
	LAT_MODE_RATES,  // roll, pitch, and yaw rate commands
	LAT_MODE_ANGLES,  // roll and pitch angle commands, yaw rate command
	LAT_MODE_COORDINATED,  // roll controls turn rate coordinated with vehicle roll, pitch controls pitch
	LAT_MODE_AUTO,  // autopilot controls all inputs
	LAT_MODE_EXTERNAL,
	LAT_MODE_INVALID,
}  __attribute__ ((packed)) LateralControlMode_t;

typedef enum {
	AUTO,
	ON,
	OFF,
}  __attribute__ ((packed)) LoopMode_t;

typedef enum {
	NAV_OFF,  // Navigation controller is not running
	NAV_POS,  // hold position and heading
	NAV_PILOT_BODY,  // track position and yaw
	NAV_PILOT_WORLD,  // track position and yaw
	NAV_PILOT_BASE,  // track position and yaw
	NAV_WPT_HCMD,  // track waypoint plan at command height
	NAV_WPT,  // follow waypoint plan (include waypoint alt)
	NAV_INVALID,
}  __attribute__ ((packed)) NavigationControllerMode_t;

typedef struct _Command_t {
	uint8_t id;
	float value;

#ifdef __cplusplus
	_Command_t() {
		id = 255;
		value = 0.0;
	}
#endif
} __attribute__ ((packed)) Command_t;

typedef struct _HandsetCalibration_t {
	uint8_t channel;
	HandsetFunction_t function_min;
	HandsetFunction_t function_mid;
	HandsetFunction_t function_max;
	HandsetType_t type;
	uint16_t usec;

#ifdef __cplusplus
	_HandsetCalibration_t() {
		channel = 255;
		function_min = HS_FUNC_INVALID;
		function_mid = HS_FUNC_INVALID;
		function_max = HS_FUNC_INVALID;
		type = HS_TYPE_INVALID;
		usec = 2000;
	}
#endif
} __attribute__ ((packed)) HandsetCalibration_t;

typedef struct _Limit_t {
	float min;
	float max;

#ifdef __cplusplus
	_Limit_t() {
		min = -INFINITY;
		max = INFINITY;
	}
#endif
} __attribute__ ((packed)) Limit_t;

typedef struct _Timeout_t {
	uint8_t seconds;
	uint8_t waypoint;

#ifdef __cplusplus
	_Timeout_t() {
		seconds = 0;
		waypoint = 0;
	}
#endif
} __attribute__ ((packed)) Timeout_t;

typedef struct _PID_t {
	uint8_t id;
	float p;
	float i;
	float d;
	Limit_t output;

#ifdef __cplusplus
	_PID_t() {
		id = 0;
		p = 0.0;
		i = 0.0;
		d = 0.0;
	}
#endif
} __attribute__ ((packed)) PID_t;

typedef struct _TabletJoystick_t {
	int16_t axis[4];  // [-1000, 1000]
	uint8_t button[2];  // bitwise position

#ifdef __cplusplus
	_TabletJoystick_t() {
		uint8_t _i;

		for (_i = 0; _i < 4; ++_i)
			axis[_i] = 0;

		for (_i = 0; _i < 2; ++_i)
			button[_i] = 0;
	}
#endif
} __attribute__ ((packed)) TabletJoystick_t;

/*--------[ Communication ]--------*/

typedef struct _TelemetryControl_t {
	float roll;
	float pitch;
	float yaw;
	float roll_rate;
	float pitch_rate;
	float yaw_rate;
	float vx;
	float vy;
	float vrate;
	float altitude;
	uint8_t waypoint;
	uint8_t look_at_point;
	LateralControlMode_t lat_mode;
	AltitudeControlMode_t alt_mode;
	NavigationControllerMode_t nav_mode;
	uint8_t landing_status;  // use LandingStatus_t to set bits
	float actuators[16];

#ifdef __cplusplus
	_TelemetryControl_t() {
		uint8_t _i;

		roll = 0.0;
		pitch = 0.0;
		yaw = 0.0;
		roll_rate = 0.0;
		pitch_rate = 0.0;
		yaw_rate = 0.0;
		vx = 0.0;
		vy = 0.0;
		vrate = 0.0;
		altitude = 0.0;
		waypoint = 0;
		look_at_point = 0;
		lat_mode = LAT_MODE_INVALID;
		alt_mode = ALT_MODE_INVALID;
		nav_mode = NAV_INVALID;
		landing_status = 0;

		for (_i = 0; _i < 16; ++_i)
			actuators[_i] = 0.0;
	}
#endif
} __attribute__ ((packed)) TelemetryControl_t;

/*--------[ Telemetry ]--------*/

typedef struct _DeploymentTube_t {
	uint8_t state;
	uint8_t parachute_door;
	uint8_t error;

#ifdef __cplusplus
	_DeploymentTube_t() {
		state = 0;
		parachute_door = 0;
		error = 0;
	}
#endif
} __attribute__ ((packed)) DeploymentTube_t;

typedef struct _TelemetryOrientation_t {
	float q[4];
	ThreeAxisSensor_t omega;
	ThreeAxisSensor_t magnetometer;  // [uT]

#ifdef __cplusplus
	_TelemetryOrientation_t() {
		uint8_t _i;

		for (_i = 0; _i < 4; ++_i)
			q[_i] = 0.0;
	}
#endif
} __attribute__ ((packed)) TelemetryOrientation_t;

typedef struct _TelemetryPosition_t {
	double latitude;  // [deg]
	double longitude;  // [deg]
	float altitude;  // [m] - mean sea level
	float height;  // [m] - above ground level
	ThreeAxisSensor_t position;  // [m]
	ThreeAxisSensor_t velocity;  // [m/s]
	ThreeAxisSensor_t acceleration;  // [m/s^2]

#ifdef __cplusplus
	_TelemetryPosition_t() {
		latitude = 0.0;
		longitude = 0.0;
		altitude = 0.0;
		height = 0.0;
	}
#endif
} __attribute__ ((packed)) TelemetryPosition_t;

typedef struct _TelemetryPressure_t {
	float static_pressure;  // [Pa]
	float dynamic_pressure;  // [Pa]
	float air_temperature;  // [deg C]
	float humidity;  // [%]
	ThreeAxisSensor_t wind;
	float ias;  // [m/s]
	float alpha;  // [rad]
	float beta;  // [rad]

#ifdef __cplusplus
	_TelemetryPressure_t() {
		static_pressure = 0.0;
		dynamic_pressure = 0.0;
		air_temperature = 0.0;
		humidity = 0.0;
		ias = 0.0;
		alpha = 0.0;
		beta = 0.0;
	}
#endif
} __attribute__ ((packed)) TelemetryPressure_t;

typedef struct _TelemetrySystem_t {
	float batt_voltage;  // [V]
	float batt_current;  // [A]
	float batt_coulomb_count;  // [Ah]
	float batt_percent;  // [%]
	float flight_time;  // [s]
	uint16_t week;
	uint8_t hour;
	uint8_t minute;
	float seconds;
	uint8_t satellites;
	float pdop;
	GPSFixType_t fix_type;
	int8_t rssi;
	uint8_t lost_comm;
	uint8_t lost_gps;
	uint8_t engine_on;
	uint32_t error_code;
	AutopilotMode_t autopilot_mode;
	FlightMode_t flight_mode;

#ifdef __cplusplus
	_TelemetrySystem_t() {
		batt_voltage = 0.0;
		batt_current = 0.0;
		batt_coulomb_count = 0.0;
		batt_percent = 0.0;
		flight_time = 0.0;
		week = 0;
		hour = 0;
		minute = 0;
		seconds = 0.0;
		satellites = 0;
		pdop = 0.0;
		rssi = 0;
		lost_comm = 0;
		lost_gps = 0;
		engine_on = 0;
		error_code = 0;
	}
#endif
} __attribute__ ((packed)) TelemetrySystem_t;

#ifdef __cplusplus
// } /* namespace default */

} /* namespace comms */
} /* namespace bst */
#endif

#endif /* _COMM_PACKETS_H_ */
