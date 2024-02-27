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

#ifndef _FIXEDWING_H_
#define _FIXEDWING_H_

#include <inttypes.h>

#ifdef __cplusplus
namespace bst {
namespace comms {
namespace fixedwing {
#endif

/*--------[ Controller ]--------*/

typedef enum {
	/* NOTE - you must check the numbers in the CommandID values */
	/* contained in the parent folder */
	/*  */
	/* Controller Modes */
	CMD_TECS_MODE=24,

	/* Velocity */
	CMD_IAS=25,

	/* System Commands */
	CMD_CAPTURE_TRIMS=26,
}  __attribute__ ((packed)) CommandID_t;

typedef enum {
	CTRL_TURNRATE_2_RUD,
	CTRL_PITCH_2_ELEVATOR,
	CTRL_ROLL_2_AIL,
	CTRL_NAV_2_ROLL,  // no I or D terms
	CTRL_ENG_2_THROTTLE,  // D is a FF term
	CTRL_ENG_2_PITCH,  // D is a FF term
	CTRL_INVALID,
}  __attribute__ ((packed)) ControlLoop_t;

typedef enum {
	TECS_MODE_OFF,
	TECS_MODE_CLIMB,
	TECS_MODE_ALT_HOLD,
	TECS_MODE_VRATE,
	TECS_MODE_LAND,
	TECS_MODE_FLARE,
}  __attribute__ ((packed)) TECSMode_t;

typedef struct _FilterParameters_t {
	float ias_alpha;  // LPF alpha value (0,1]
	float ias_dot_alpha;  // LPF alpha value (0,1]
	float gamma_alpha;  // LPF alpha value (0,1]
	float roll_cmd_rate;  // rate limit for roll cmd [rad/s]: default=1.57 (90 deg/s)
	float pitch_cmd_rate;  // rate limit for pitch cmd [rad/s]: default=0.524 (30 deg/s)
	float ias_cmd_rate;  // rate limit for ias cmd [m/s^2]: default=1
	float gamma_cmd_rate;  // rate limit for gamma cmd [rad/s]: default=0.34
	float throttle_cmd_rate;  // rate limit for throttle cmd [/s]: default=2
	float k_cmd_rate;  // rate limit for TECS K-value cmd [/s]: default=1
	float vx_dot_cmd_rate;  // rate limit for TECS dVx/dt cmd [m/2^3]
	/* need to adjust param.h if need more space */

#ifdef __cplusplus
	_FilterParameters_t() {
		ias_alpha = 0.0;
		ias_dot_alpha = 0.0;
		gamma_alpha = 0.0;
		roll_cmd_rate = 0.0;
		pitch_cmd_rate = 0.0;
		ias_cmd_rate = 0.0;
		gamma_cmd_rate = 0.0;
		throttle_cmd_rate = 0.0;
		k_cmd_rate = 0.0;
		vx_dot_cmd_rate = 0.0;
	}
#endif
} __attribute__ ((packed)) FilterParameters_t;

typedef struct _FlightControlParameters_t {
	float tecs_Kv;
	float tecs_Kh;
	float tecs_max_vx_dot;  // [m/s^2]
	float min_ground_speed;  // [m/s]
	float nav_lookahead;  // [s]
	float min_nav_lookahead_dist;  // [m]
	float tuning_ias;  // [m/s]
	float max_height_error_mode;  // max height to switch to new k_height_tracking: default=20
	float max_v_error_mode;  // max speed to switch to k_speed_hold: default=3
	float k_height_tracking;  // K-value with large height error: default=1.75
	float k_flare;  // K-value during flare: default=0.5
	float k_land;  // K-value during landing; default=1.5
	float k_cruise;  // K-value for altitude hold mode: default=0.5
	float k_climb;  // K-value for climbout mode: default=1.8
	float k_speed_hold;  // K-value with large speed error
	/* need to adjust param.h if need more space */

#ifdef __cplusplus
	_FlightControlParameters_t() {
		tecs_Kv = 0.0;
		tecs_Kh = 0.0;
		tecs_max_vx_dot = 0.0;
		min_ground_speed = 0.0;
		nav_lookahead = 0.0;
		min_nav_lookahead_dist = 0.0;
		tuning_ias = 0.0;
		max_height_error_mode = 0.0;
		max_v_error_mode = 0.0;
		k_height_tracking = 0.0;
		k_flare = 0.0;
		k_land = 0.0;
		k_cruise = 0.0;
		k_climb = 0.0;
		k_speed_hold = 0.0;
	}
#endif
} __attribute__ ((packed)) FlightControlParameters_t;

typedef struct _SurfaceMixing_t {
	float mixing_roll_2_elevator;  // [-1,1]
	float mixing_aileron_2_rudder;  // [-1,1]
	float mixing_flap_2_elevator;  // [-1,1]

#ifdef __cplusplus
	_SurfaceMixing_t() {
		mixing_roll_2_elevator = 0.0;
		mixing_aileron_2_rudder = 0.0;
		mixing_flap_2_elevator = 0.0;
	}
#endif
} __attribute__ ((packed)) SurfaceMixing_t;

/*--------[ System ]--------*/

typedef enum {
	ORIENTATION_INITIALIZED=1,
	ACTUATORS_INITIALIZED=2,
	PARAM_INITIALIZED=4,
	VECHILE_LIMITS_INITIALIZED=8,
	CONTROL_GAINS_INITIALIZED=16,
	LAUNCH_PARAM_INITIALIZED=32,
	LANDING_PARAM_INITIALIZED=64,
	SENSORS_INITIALIZED=128,
}  __attribute__ ((packed)) VehicleInitialization_t;

/*--------[ Vehicle ]--------*/

typedef enum {
	LAND_SPIRAL,
}  __attribute__ ((packed)) LandType_t;

typedef enum {
	LANDING_INVALID,
	LANDING_STATUS_ENTER,
	LANDING_STATUS_TRACKING,
	LANDING_STATUS_HOLDING,
	LANDING_STATUS_FINAL,
	LANDING_STATUS_SHORT,
	LANDING_STATUS_LONG,
	LANDING_STATUS_LATERAL,
	LANDING_STATUS_MANUAL,
	LANDING_STATUS_COMMITTED,
}  __attribute__ ((packed)) LandingStatus_t;

typedef enum {
	LAUNCH_HAND,
	LAUNCH_BUNGEE,
	LAUNCH_WINCH,
	LAUNCH_ROLLING,
	LAUNCH_CAR,
	LAUNCH_RAIL,
	LAUNCH_DROP,
}  __attribute__ ((packed)) LaunchType_t;

typedef struct _LandingParameters_t {
	float ias;  // [m/s]
	float glide_slope;  // [rad] flight path angle
	float safe_height;  // [m] min safe altitude for manuevers
	float abort_height;  // [m] altitude for abort orbit
	float flap_deflection;  // [frac] how much flaps to use
	float flare_min_pitch;  // [rad] min pitch angle during flare
	float cross_track_error;  // [m] error at touchdown point
	float cross_track_angle;  // [rad] total error is error plus angle growth
	float height_error_bound;  // [m] bound on error for low tracking to generate short abort
	float abort_trigger_time;  // [s] how long a trigger must exist before an abort
	float decision_time;  // [s] when to start checking for abort conditions
	float commit_time;  // [s] time at which plane commits to landing
	float flare_time;  // [s] when to initiate flare based on time-to-impact
	float agl_offset;  // [m] agl sensor offset for landing gear-sensor location
	/* -- */
	uint8_t unused[12];  // place holder for future parameters
	/* need to adjust param.h if need more space */

#ifdef __cplusplus
	_LandingParameters_t() {
		uint8_t _i;

		ias = 0.0;
		glide_slope = 0.0;
		safe_height = 0.0;
		abort_height = 0.0;
		flap_deflection = 0.0;
		flare_min_pitch = 0.0;
		cross_track_error = 0.0;
		cross_track_angle = 0.0;
		height_error_bound = 0.0;
		abort_trigger_time = 0.0;
		decision_time = 0.0;
		commit_time = 0.0;
		flare_time = 0.0;
		agl_offset = 0.0;

		for (_i = 0; _i < 12; ++_i)
			unused[_i] = 0;
	}
#endif
} __attribute__ ((packed)) LandingParameters_t;

typedef struct _LaunchParameters_t {
	float ias;  // [m/s]
	float flap_deflection;  // [%]
	float throttle_delay;  // [s]
	float throttle_setting;  // [%]
	float min_pitch;  // [rad]
	float climbout_angle;  // [rad]
	float climbout_height;  // [m]
	float timeout;  // [s] max time in climbout mode
	float elevator_deflection;  // [%] elevator position release
	/* -- */
	uint8_t unused[28];  // place holder for future parameters
	/* need to adjust param.h if need more space */

#ifdef __cplusplus
	_LaunchParameters_t() {
		uint8_t _i;

		ias = 0.0;
		flap_deflection = 0.0;
		throttle_delay = 0.0;
		throttle_setting = 0.0;
		min_pitch = 0.0;
		climbout_angle = 0.0;
		climbout_height = 0.0;
		timeout = 0.0;
		elevator_deflection = 0.0;

		for (_i = 0; _i < 28; ++_i)
			unused[_i] = 0;
	}
#endif
} __attribute__ ((packed)) LaunchParameters_t;

typedef struct _VehicleLimits_t {
	Limit_t roll_angle;  // [rad] also kept in control loops
	Limit_t pitch_angle;  // [rad] also kept in control loops
	Limit_t ias;  // [m/s]
	uint8_t lost_gps;  // how long till we consider GPS lost
	float lost_gps_roll;  // [rad]
	float max_pdop;  // max pdop to trust GPS
	float max_scale_factor;  // scale factor for IAS based gain schedule: default=5
	Limit_t flightpath_angle;  // [rad] [-1,1]
	Limit_t flightpath_angle_flap;  // [rad], (-1, 1)
	float flightpath_angle_fraction;  // [percent] [0,1]
	/* -- */
	uint8_t unused[10];  // place holder for future parameters
	/* need to adjust param.h if need more space */

#ifdef __cplusplus
	_VehicleLimits_t() {
		uint8_t _i;

		lost_gps = 0;
		lost_gps_roll = 0.0;
		max_pdop = 0.0;
		max_scale_factor = 0.0;
		flightpath_angle_fraction = 0.0;

		for (_i = 0; _i < 10; ++_i)
			unused[_i] = 0;
	}
#endif
} __attribute__ ((packed)) VehicleLimits_t;

typedef struct _VehicleParameters_t {
	char name[16];  // used to set the name of the vehicle
	float flight_time;  // [min]
	float standard_bank;  // [rad]
	float cruise_speed;  // [m/s]
	float battery_cap;  // [Wh]
	uint8_t battery_num_cells;  //
	BatteryChemistry_t batt_chem;  // set the type of battery
	/* :RadioType radio_type # set the type of radio used on serial port */
	uint8_t unused[32];  // place holder for future parameters
	/* need to adjust param.h if need more space */

#ifdef __cplusplus
	_VehicleParameters_t() {
		uint8_t _i;

		for (_i = 0; _i < 16; ++_i)
			name[_i] = 0;

		flight_time = 0.0;
		standard_bank = 0.0;
		cruise_speed = 0.0;
		battery_cap = 0.0;
		battery_num_cells = 0;

		for (_i = 0; _i < 32; ++_i)
			unused[_i] = 0;
	}
#endif
} __attribute__ ((packed)) VehicleParameters_t;

/*--------[ Mission ]--------*/

typedef struct _MissionParameters_t {
	Limit_t altitude;  // [m]
	Timeout_t comm;
	LaunchType_t launch_type;  // set the type of lauch used
	LandType_t land_type;  // set the type of landing used
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

#ifdef __cplusplus
} /* namespace fixedwing */

} /* namespace comms */
} /* namespace bst */
#endif

#endif /* _FIXEDWING_H_ */
