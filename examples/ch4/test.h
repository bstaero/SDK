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

typedef struct _OldTelemetryPosition_t {
	float latitude;  // [deg]
	float longitude;  // [deg]
	float altitude;  // [m]
	float height;  // [m]
	float speed;  // [m/s]
	float course;  // [deg] 
	ThreeAxisSensor_t position;  // [m]
	ThreeAxisSensor_t velocity;  // [m/s]
	ThreeAxisSensor_t acceleration;  // [m/s^2]

#ifdef __cplusplus
	_OldTelemetryPosition_t() {
		latitude = 0.0;
		longitude = 0.0;
		altitude = 0.0;
		height = 0.0;
	}
#endif
} __attribute__ ((packed)) OldTelemetryPosition_t;

extern volatile bool write_file;
extern volatile bool simulation;

void initializeTest(void);
void updateTest(void);
void exitTest(void);
void printTestHelp(void);

#endif
