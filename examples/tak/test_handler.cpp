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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "test.h"
#include "main.h"
#include "structs.h"

std::map<uint32_t, Aircraft_t> aircraft;
std::map<uint32_t, SourceStats_t> rx_sources;

// BST source addresses: 0x41xxxxxx aircraft, 0x52xxxxxx / 0x53xxxxxx ground
// stations. Anything else (e.g. no addressing on the link) is treated as an
// aircraft so a direct serial connection still works.
static bool isAircraft(uint32_t address) {
	uint32_t prefix = address & 0xFF000000;
	return prefix != 0x52000000 && prefix != 0x53000000;
}

static Aircraft_t & getAircraft(uint32_t address) {
	std::map<uint32_t, Aircraft_t>::iterator it = aircraft.find(address);
	if(it != aircraft.end()) return it->second;

	// operator[] value-initializes, so name/flags start zeroed
	Aircraft_t & ac = aircraft[address];
	ac.last_tak_tx = -1e6;
	printf("New aircraft: 0x%08X\n", address);
	return ac;
}

void receive(uint8_t type, void * data, uint16_t size, const void * parameter)
{
	uint32_t address = ((BSTProtocol *)comm_handler)->getLastAddress();

	SourceStats_t & src = rx_sources[address];
	src.packets++;
	src.types.insert(type);

	switch(type) {
		case TELEMETRY_POSITION: {       // TelemetryPosition_t
			if(!isAircraft(address) || size < sizeof(TelemetryPosition_t)) break;

			Aircraft_t & ac = getAircraft(address);
			memcpy(&ac.position, data, sizeof(TelemetryPosition_t));
			ac.have_position = true;
			ac.last_rx = getElapsedTime();
			break;
		}

		case TELEMETRY_SYSTEM:           // TelemetrySystem_t
			if(!isAircraft(address) || size < sizeof(TelemetrySystem_t)) break;
			memcpy(&getAircraft(address).system, data, sizeof(TelemetrySystem_t));
			break;

		case SYSTEM_INITIALIZE: {        // SystemInitialize_t - carries the aircraft name
			if(!isAircraft(address) || size < sizeof(SystemInitialize_t)) break;

			SystemInitialize_t * init = (SystemInitialize_t *)data;
			Aircraft_t & ac = getAircraft(address);
			memcpy(ac.name, init->name, 16);
			ac.name[16] = '\0';
			break;
		}

		default:
			break;
	}
}

uint8_t receiveCommand(uint8_t type, void * data, uint16_t size, const void * parameter)
{
	// read-only example, never accept commands
	return false;
}

void receiveReply(uint8_t type, void * data, uint16_t size, bool ack, const void * parameter)
{
}

bool publish(uint8_t type, uint8_t param)
{
	return false;
}
