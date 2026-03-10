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
#ifndef _TEST_H_
#define _TEST_H_

#include <inttypes.h>

#include "structs.h"

typedef enum {
	LDCR_STATUS_OFF,
	LDCR_STATUS_CALIBRATE_CAL_TARGET,
	LDCR_STATUS_CALIBRATE_WATER,
	LDCR_STATUS_CALIBRATE_COLD_SPACE,
	LDCR_STATUS_RECORDING,
	LDCR_STATUS_INVALID,
}  __attribute__ ((packed)) LDCRStatus_t;

typedef struct _LDCRCommand_t {
	LDCRStatus_t sensor_command;

#ifdef __cplusplus
	_LDCRCommand_t() {
		sensor_command = LDCR_STATUS_INVALID;
	}
#endif
} __attribute__ ((packed)) LDCRCommand_t;

typedef struct _LDCRTelemetry_t {
	LDCRStatus_t sensor_status;
	float time;
	double latitude;
	double longitude;
	float altitude;
	float height;
	float ndvi;
	float ground_temperature;
	float brightness_temperature;

#ifdef __cplusplus
	_LDCRTelemetry_t() {
		sensor_status = LDCR_STATUS_INVALID;
		time = 0;
		latitude = 0.0;
		longitude = 0.0;
		altitude = 0.0;
		height = 0.0;
		ndvi = 0.0;
		ground_temperature = 0.0;
		brightness_temperature = 0.0;
	}
#endif
} __attribute__ ((packed)) LDCRTelemetry_t;

void updateTest(void);
void printTestHelp(void);

void publishPayloadCommand(uint8_t channel);
bool setPayloadCommand(uint8_t channel, UserPayload_t * data);

#endif
