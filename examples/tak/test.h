/*=+--+=#=+--         SwiftCore Flight Management Software        --+=#=+--+=#*\
|               Copyright (C) 2026 Black Swift Technologies LLC.               |
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
#include <map>

#include "structs.h"
#include "tak_client.h"

// One entry per aircraft seen through the ground station, keyed by the
// BST packet source address (0x41xxxxxx for aircraft).
typedef struct {
	char               name[17];
	TelemetryPosition_t position;
	TelemetrySystem_t   system;
	bool               have_position;
	float              last_rx;      // [s] last TELEMETRY_POSITION
	float              last_tak_tx;  // [s] last CoT sent
} Aircraft_t;

extern std::map<uint32_t, Aircraft_t> aircraft;

extern TakClient tak;
extern float     tak_period;  // [s] minimum time between CoT updates per aircraft
extern float     tak_stale;   // [s] CoT stale time
extern const char * tak_type; // CoT type for aircraft

void updateTest(void);
void printTestHelp(void);

#endif
