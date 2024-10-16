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

#ifndef _MULTICOPTER_H_
#define _MULTICOPTER_H_

#include <inttypes.h>

#ifdef __cplusplus
namespace bst {
namespace comms {
namespace multicopter {
#endif

/*--------[ Actuators ]--------*/

#define MAX_NUM_ROTORS 16

/*--------[ Controller ]--------*/

typedef enum {
	/* NOTE - you must check the numbers in the CommandID values */
	/* contained in the parent folder */
	/*  */

	/* Composite */
	CMD_VEL_CTRL=26,

	/* Position */
	CMD_X_POS=27,
	CMD_Y_POS=28,
	CMD_X_VEL=29,
	CMD_Y_VEL=30,

	/* Thrust/Moment Commands */
	CMD_THRUST=31,
	CMD_MOMENT_X=32,
	CMD_MOMENT_Y=33,
	CMD_MOMENT_Z=34,

	/* Speed */
	CMD_SOG=25,
}  __attribute__ ((packed)) CommandID_t;

typedef enum {
	CTRL_ANG_TO_RATE,  // Angle error to rate
	CTRL_ROLL_RATE,  // PID on roll rate
	CTRL_PITCH_RATE,  // PID on pitch rate
	CTRL_YAW_RATE,  // PID on yaw rate
	CTRL_X_VEL_TO_ACC,  // x velocity error to acceleration
	CTRL_Y_VEL_TO_ACC,  // y velocity error to acceleration
	CTRL_POS,  // PID on horizontal speed
	CTRL_ALT,  // PID on vrate
	CTRL_THRUST,  // PID on thrust
	CTRL_INVALID,
}  __attribute__ ((packed)) ControlLoop_t;

typedef struct _FlightControlParameters_t {
	float min_ground_speed;  // [m/s]
	float nav_lookahead;  // [s]
	float min_nav_lookahead_dist;  // [m]
	float wpt_capture_dist;  // [m] radius of circle when to hold position
	/* -- */
	uint8_t unused[32];  // place holder for future parameters
	/* need to adjust memory param table if need more space */

#ifdef __cplusplus
	_FlightControlParameters_t() {
		uint8_t _i;

		min_ground_speed = 0.0;
		nav_lookahead = 0.0;
		min_nav_lookahead_dist = 0.0;
		wpt_capture_dist = 0.0;

		for (_i = 0; _i < 32; ++_i)
			unused[_i] = 0;
	}
#endif
} __attribute__ ((packed)) FlightControlParameters_t;

typedef struct _VehicleLimits_t {
	Limit_t roll;  // [rad]
	Limit_t pitch;  // [rad]
	float roll_rate;  // [rad/s]
	float pitch_rate;  // [rad/s]
	float yaw_rate;  // [rad/s]
	float speed;  // [m/s] side speed limit
	Limit_t vrate;  // [m/s] vertical speed limit
	uint8_t lost_gps;  // how long till we consider GPS lost
	float max_pdop;  // max pdop to trust GPS

	/* -- */
	uint8_t unused[24];  // place holder for future parameters
	/* need to adjust memory param table if need more space */

#ifdef __cplusplus
	_VehicleLimits_t() {
		uint8_t _i;

		roll_rate = 0.0;
		pitch_rate = 0.0;
		yaw_rate = 0.0;
		speed = 0.0;
		lost_gps = 0;
		max_pdop = 0.0;

		for (_i = 0; _i < 24; ++_i)
			unused[_i] = 0;
	}
#endif
} __attribute__ ((packed)) VehicleLimits_t;

/*--------[ Logging ]--------*/

typedef struct _LogFlightControl_t {
	/* This packet defines values only logged on board vehicle */
	uint8_t data[64];

#ifdef __cplusplus
	_LogFlightControl_t() {
		uint8_t _i;

		for (_i = 0; _i < 64; ++_i)
			data[_i] = 0;
	}
#endif
} __attribute__ ((packed)) LogFlightControl_t;

/*--------[ Mission ]--------*/

typedef struct _MissionParameters_t {
	Limit_t altitude;  // [m]
	Timeout_t comm;
	float max_range;  // [m] max distance from base station
	float safe_height;  // [m] min safe altitude for manuevers
	float flight_time;  // [min] flight time for terminating mission
	float battery_min;  // [%] min battery percent for terminiating mission
	uint8_t initialized;  // bitwise field of init systems
	float mag_dec;  // [rad] magnetic declination at local area

	/* -- */
	uint8_t unused[16];  // place holder for future parameters
	/* need to adjust param.h if need more space */

#ifdef __cplusplus
	_MissionParameters_t() {
		uint8_t _i;

		max_range = 0.0;
		safe_height = 0.0;
		flight_time = 0.0;
		battery_min = 0.0;
		initialized = 0;
		mag_dec = 0.0;

		for (_i = 0; _i < 16; ++_i)
			unused[_i] = 0;
	}
#endif
} __attribute__ ((packed)) MissionParameters_t;

/*--------[ Rotors ]--------*/

typedef enum {
	ROTOR_DIR_CW,
	ROTOR_DIR_CCW,
	ROTOR_DIR_INVALID,
}  __attribute__ ((packed)) RotorDir_t;

typedef struct _RotorParameters_t {
	uint8_t id;  // Rotor ID
	uint8_t channel;  // Rotors output channel
	float k_wv;  // [RPM/V/usec] - linear gain of RPM to voltage and PWM
	float pwm_o;  // [usec] - zero point of ESC
	float pos_x;  // [m] - Motor position vector
	float pos_y;  // [m]
	float pos_z;  // [m]
	float t_x;  // Thrust direction unit vector
	float t_y;
	float t_z;
	RotorDir_t dir;  // direction of rotor
	float rpm_to_thrust;  // [N/rpm^2] - max thrust
	float thrust_to_moment;  // [N-m/N] - constant prop thrust to moment

#ifdef __cplusplus
	_RotorParameters_t() {
		id = 0;
		channel = 0;
		k_wv = 0.0;
		pwm_o = 0.0;
		pos_x = 0.0;
		pos_y = 0.0;
		pos_z = 0.0;
		t_x = 0.0;
		t_y = 0.0;
		t_z = 0.0;
		dir = ROTOR_DIR_INVALID;
		rpm_to_thrust = 0.0;
		thrust_to_moment = 0.0;
	}
#endif
} __attribute__ ((packed)) RotorParameters_t;

/*--------[ System ]--------*/

typedef enum {
	ORIENTATION_INITIALIZED=1,
	ACTUATORS_INITIALIZED=2,
	PARAM_INITIALIZED=4,
	VECHILE_LIMITS_INITIALIZED=8,
	CONTROL_GAINS_INITIALIZED=16,
	LAUNCH_PARAM_INITIALIZED=32,
	LANDING_PARAM_INITIALIZED=64,
	ROTORS_INITIALIZED=128,
	ESTIMATOR_INITIALIZED=256,
	FILTERS_INITIALIZED=512,
	SENSORS_INITIALIZED=1024,
}  __attribute__ ((packed)) VehicleInitialization_t;

/*--------[ Vehicle ]--------*/

typedef enum {
	LANDING_INVALID,
	LANDING_STATUS_ENTER,
	LANDING_STATUS_TRACKING,
	LANDING_STATUS_HOLDING,
	LANDING_STATUS_FINAL,
	LANDING_STATUS_MANUAL,
	LANDING_STATUS_COMMITTED,
}  __attribute__ ((packed)) LandingStatus_t;

typedef struct _LandingParameters_t {
	float safe_height;  // [m] min safe altitude for manuevers
	float descend_rate;  // [m/s] descend rate
	float agl_offset;  // [m] agl sensor offset for landing gear-sensor location

	/* -- */
	uint8_t unused[16];  // place holder for future parameters
	/* need to adjust memory param table if need more space */

#ifdef __cplusplus
	_LandingParameters_t() {
		uint8_t _i;

		safe_height = 0.0;
		descend_rate = 0.0;
		agl_offset = 0.0;

		for (_i = 0; _i < 16; ++_i)
			unused[_i] = 0;
	}
#endif
} __attribute__ ((packed)) LandingParameters_t;

typedef struct _LaunchParameters_t {
	/* Climbout */
	float climbout_height;  // [m]
	float timeout;  // [s] max time in climbout mode

	/* -- */
	uint8_t unused[16];  // place holder for future parameters
	/* need to adjust memory param table if need more space */

#ifdef __cplusplus
	_LaunchParameters_t() {
		uint8_t _i;

		climbout_height = 0.0;
		timeout = 0.0;

		for (_i = 0; _i < 16; ++_i)
			unused[_i] = 0;
	}
#endif
} __attribute__ ((packed)) LaunchParameters_t;

typedef struct _VehicleParameters_t {
	char name[16];  // name of the vehicle
	float battery_cap;  // [Wh]
	uint8_t battery_num_cells;  //
	BatteryChemistry_t batt_chem;  // set the type of battery
	uint8_t num_rotors;  // [#]
	float mass;  // [kg]
	float torque;  // torque due to drag [N / (m/s)]
	float drag;  // drag on the vehicle [N]
	float cruise_speed;  // [m/s]

	/* -- */
	uint8_t unused[12];  // place holder for future parameters
	/* need to adjust memory param table if need more space */

#ifdef __cplusplus
	_VehicleParameters_t() {
		uint8_t _i;

		for (_i = 0; _i < 16; ++_i)
			name[_i] = 0;

		battery_cap = 0.0;
		battery_num_cells = 0;
		num_rotors = 0;
		mass = 0.0;
		torque = 0.0;
		drag = 0.0;
		cruise_speed = 0.0;

		for (_i = 0; _i < 12; ++_i)
			unused[_i] = 0;
	}
#endif
} __attribute__ ((packed)) VehicleParameters_t;

#ifdef __cplusplus
} /* namespace multicopter */

} /* namespace comms */
} /* namespace bst */
#endif

#endif /* _MULTICOPTER_H_ */
