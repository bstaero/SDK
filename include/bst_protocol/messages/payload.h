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

#ifndef _PAYLOAD_H_
#define _PAYLOAD_H_

#include <inttypes.h>

#ifdef __cplusplus
namespace bst {
namespace comms {
namespace payload {
#endif

/*--------[ Actuators ]--------*/

#define NUM_PAYLOAD_CHANNELS 8

#define NUM_PAYLOAD_DATA_CHANNELS 8

/*--------[ Controller ]--------*/

typedef enum {
	/* NOTE - you must check the numbers in the CommandID values */
	/* contained in the parent folder */
	/*  */

	/* Command Interface */
	CMD_PAYLOAD_CONTROL=35,
	/* Gimbal Control */
	CMD_LOOK_AT=36,
}  __attribute__ ((packed)) CommandID_t;

/*--------[ Input ]--------*/

typedef enum {
	INTERFACE_UNKNOWN,
	INTERFACE_BST_PROTOCOL,
	INTERFACE_UBLOX_GPS,
	INTERFACE_NMEA_GPS,
	INTERFACE_MAVLINK,
	INTERFACE_PAYLOAD_PASSTHRU,
	INTERFACE_HWIL,
}  __attribute__ ((packed)) PayloadInterface_t;

typedef enum {
	UNKNOWN_TYPE,
	DISCRETE_IO,
	PWM_50HZ,
	PWM_300HZ,
	FALLING_EDGE_TRIGGER,
	RISING_EDGE_TRIGGER,
	CONTINUOUS_TRIGGER,
	CONTINUOUS_TRIGGER_LOW,
}  __attribute__ ((packed)) PayloadSignal_t;

typedef enum {
	PAYLOAD_TYPE_CAMERA,
	PAYLOAD_TYPE_DOOR,
	PAYLOAD_TYPE_POWER,
	PAYLOAD_TYPE_PITCH,
	PAYLOAD_TYPE_ROLL,
	PAYLOAD_TYPE_YAW,
	PAYLOAD_TYPE_UNUSED,
}  __attribute__ ((packed)) PayloadType_t;

/*--------[ PAYLOAD ]--------*/

typedef enum {
	PAYLOAD_CTRL_OFF,
	PAYLOAD_CTRL_CONNECTED,
	PAYLOAD_CTRL_READY,
	PAYLOAD_CTRL_ACTIVE,
	PAYLOAD_CTRL_SHUTDOWN,
	PAYLOAD_CTRL_ERROR,
	PAYLOAD_CTRL_INVALID,
}  __attribute__ ((packed)) PayloadControl_t;

typedef enum {
	PAYLOAD_STATE_OPEN,
	PAYLOAD_STATE_CLOSED,
	PAYLOAD_STATE_AUTO,
	PAYLOAD_STATE_ON,
	PAYLOAD_STATE_OFF,
	PAYLOAD_STATE_MANUAL,
	PAYLOAD_STATE_UNKNOWN,
}  __attribute__ ((packed)) PayloadState_t;

/*--------[ Payload Sensors ]--------*/

#define PARTICLESPLUS_MAX_CHANNELS 6

typedef enum {
	PLATFORM_TYPE_UAS,
	PLATFORM_TYPE_PORTABLE,
	PLATFORM_TYPE_FIXED,
}  __attribute__ ((packed)) LDCRPlatformType_t;

typedef struct _K30_t {
	float system_time;
	uint16_t co2;
	uint16_t temp;

#ifdef __cplusplus
	_K30_t() {
		system_time = 0.0;
		co2 = 0;
		temp = 0;
	}
#endif
} __attribute__ ((packed)) K30_t;

typedef struct _MiniGAS_t {
	float system_time;
	float gas01_mv;
	float gas01_ppm;
	float gas02_mv;
	float gas02_ppm;
	float gas03_mv;
	float gas03_ppm;
	float gas04_mv;
	float gas04_ppm;
	float co2_ppm;
	float h20_hpa;
	float co2_int_temp;
	float air_temp;
	float logger_temp;
	float pressure;

#ifdef __cplusplus
	_MiniGAS_t() {
		system_time = 0.0;
		gas01_mv = 0.0;
		gas01_ppm = 0.0;
		gas02_mv = 0.0;
		gas02_ppm = 0.0;
		gas03_mv = 0.0;
		gas03_ppm = 0.0;
		gas04_mv = 0.0;
		gas04_ppm = 0.0;
		co2_ppm = 0.0;
		h20_hpa = 0.0;
		co2_int_temp = 0.0;
		air_temp = 0.0;
		logger_temp = 0.0;
		pressure = 0.0;
	}
#endif
} __attribute__ ((packed)) MiniGAS_t;

typedef struct _ParticlesPlusChannel_t {
	uint8_t channel_size;
	uint16_t differential_counts;
	float differential_counts_m;
	float differential_mass;

#ifdef __cplusplus
	_ParticlesPlusChannel_t() {
		channel_size = 0;
		differential_counts = 0;
		differential_counts_m = 0.0;
		differential_mass = 0.0;
	}
#endif
} __attribute__ ((packed)) ParticlesPlusChannel_t;

typedef struct _LDCR_t {
	uint8_t header[2];
	uint16_t serial_number;
	uint8_t hw_revision;
	uint32_t sw_revision;
	LDCRPlatformType_t platform_type;
	uint16_t platform_serial;
	uint32_t system_time;
	uint8_t calibration_state;
	int32_t sum_data[2];
	uint64_t sum_of_squares[2];
	uint16_t thermistor[8];
	uint16_t thermistor_ref;
	uint16_t week;
	uint8_t hour;
	uint8_t minute;
	float seconds;
	double latitude;  // [deg]
	double longitude;  // [deg]
	float altitude;  // [m]
	float agl;  // [m]
	float roll;  // [rad]
	float pitch;  // [rad]
	uint16_t crc;

#ifdef __cplusplus
	_LDCR_t() {
		uint8_t _i;

		for (_i = 0; _i < 2; ++_i)
			header[_i] = 0;

		serial_number = 0;
		hw_revision = 0;
		sw_revision = 0;
		platform_serial = 0;
		system_time = 0;
		calibration_state = 0;

		for (_i = 0; _i < 2; ++_i)
			sum_data[_i] = 0;

		for (_i = 0; _i < 2; ++_i)
			sum_of_squares[_i] = 0;

		for (_i = 0; _i < 8; ++_i)
			thermistor[_i] = 0;

		thermistor_ref = 0;
		week = 0;
		hour = 0;
		minute = 0;
		seconds = 0.0;
		latitude = 0.0;
		longitude = 0.0;
		altitude = 0.0;
		agl = 0.0;
		roll = 0.0;
		pitch = 0.0;
		crc = 0;
	}
#endif
} __attribute__ ((packed)) LDCR_t;

typedef struct _ParticlesPlus_t {
	float system_time;
	char date[11];
	char time[9];
	float duration;
	float sample_flow_rate;
	uint16_t sample_status_bits;
	float bp;
	ParticlesPlusChannel_t channel_data[6];

#ifdef __cplusplus
	_ParticlesPlus_t() {
		uint8_t _i;

		system_time = 0.0;

		for (_i = 0; _i < 11; ++_i)
			date[_i] = 0;

		for (_i = 0; _i < 9; ++_i)
			time[_i] = 0;

		duration = 0.0;
		sample_flow_rate = 0.0;
		sample_status_bits = 0;
		bp = 0.0;
	}
#endif
} __attribute__ ((packed)) ParticlesPlus_t;

/*--------[ Communication ]--------*/

typedef struct _TelemetryPayload_t {
	PayloadControl_t node_status;  // status of external payload node
	uint16_t num_triggers;  // if we are mapping, number of triggers
	float percent_complete;  // if we are mapping, percent path complete

#ifdef __cplusplus
	_TelemetryPayload_t() {
		num_triggers = 0;
		percent_complete = 0.0;
	}
#endif
} __attribute__ ((packed)) TelemetryPayload_t;

/*--------[ Payload ]--------*/

typedef enum {
	PAYLOAD_UNKNOWN=192,
	PAYLOAD_QX1=193,
	PAYLOAD_A6000=194,
	PAYLOAD_FLIR_TAU2=195,
	PAYLOAD_TETRACAM_ADC_LITE=196,
	PAYLOAD_A5100=197,
	PAYLOAD_MAPIR_KERNEL=198,
	PAYLOAD_FLIR_VUE_PRO=199,
	PAYLOAD_MICASENSE_REDEDGE=200,
	PAYLOAD_PARTICLES_PLUS=201,
	PAYLOAD_K30=202,
	PAYLOAD_MINIGAS=203,
	PAYLOAD_TRACE_GAS=204,
	PAYLOAD_LICOR=205,
	PAYLOAD_SPECTROMETER=206,
}  __attribute__ ((packed)) PayloadID_t;

typedef struct _NDVI_t {
	float system_time;  // [s]
	uint8_t id;  // 0=Down, 1=Up
	float red;
	float near_ir;
	float ir_ambient;  // [deg C]
	float ir_object;  // [deg C]

#ifdef __cplusplus
	_NDVI_t() {
		system_time = 0.0;
		id = 0;
		red = 0.0;
		near_ir = 0.0;
		ir_ambient = 0.0;
		ir_object = 0.0;
	}
#endif
} __attribute__ ((packed)) NDVI_t;

typedef struct _PayloadParam_t {
	uint8_t channel;  // [0-15]
	char channelName[32];
	float deltaD;  // [m]
	float pulse;  // [s]
	float powerUp;  // [s]
	float powerDown;  // [s]
	PayloadType_t payloadType;
	PayloadSignal_t payloadSignal;
	PayloadState_t payloadState;

#ifdef __cplusplus
	_PayloadParam_t() {
		uint8_t _i;

		channel = 0;

		for (_i = 0; _i < 32; ++_i)
			channelName[_i] = 0;

		deltaD = 0.0;
		pulse = 0.0;
		powerUp = 0.0;
		powerDown = 0.0;
	}
#endif
} __attribute__ ((packed)) PayloadParam_t;

typedef struct _PayloadSerial_t {
	uint32_t baudRate;
	PayloadInterface_t payloadInterface;

#ifdef __cplusplus
	_PayloadSerial_t() {
		baudRate = 0;
	}
#endif
} __attribute__ ((packed)) PayloadSerial_t;

typedef struct _PayloadTrigger_t {
	double latitude;  // [deg]
	double longitude;  // [deg]
	float altitude;  // [m] - mean sea level
	float q[4];
	uint8_t percent;
	uint16_t id;  // [#] trigger (photo) number
	uint8_t channel;  // [#] payload channel

#ifdef __cplusplus
	_PayloadTrigger_t() {
		uint8_t _i;

		latitude = 0.0;
		longitude = 0.0;
		altitude = 0.0;

		for (_i = 0; _i < 4; ++_i)
			q[_i] = 0.0;

		percent = 0;
		id = 0;
		channel = 0;
	}
#endif
} __attribute__ ((packed)) PayloadTrigger_t;

typedef struct _UserPayload_t {
	uint8_t system_id;
	uint8_t size;
	uint8_t buffer[64];

#ifdef __cplusplus
	_UserPayload_t() {
		uint8_t _i;

		system_id = 0;
		size = 0;

		for (_i = 0; _i < 64; ++_i)
			buffer[_i] = 0;
	}
#endif
} __attribute__ ((packed)) UserPayload_t;

typedef struct _CameraTag_t {
	PayloadTrigger_t trigger_info;
	uint16_t week;
	uint8_t hour;
	uint8_t minute;
	float seconds;
	/* uint8 was_manual */
	char filename[32];

#ifdef __cplusplus
	_CameraTag_t() {
		uint8_t _i;

		week = 0;
		hour = 0;
		minute = 0;
		seconds = 0.0;

		for (_i = 0; _i < 32; ++_i)
			filename[_i] = 0;
	}
#endif
} __attribute__ ((packed)) CameraTag_t;

typedef struct _PayloadStatus_t {
	PayloadID_t identifier;  // type of hardware
	uint8_t power_on;
	uint8_t initialized;
	PayloadControl_t state;

#ifdef __cplusplus
	_PayloadStatus_t() {
		identifier = PAYLOAD_UNKNOWN;
		power_on = 0;
		initialized = 0;
	}
#endif
} __attribute__ ((packed)) PayloadStatus_t;

#ifdef __cplusplus
} /* namespace payload */

} /* namespace comms */
} /* namespace bst */
#endif

#endif /* _PAYLOAD_H_ */
