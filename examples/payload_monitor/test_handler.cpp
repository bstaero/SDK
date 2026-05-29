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
//
// Packet hooks for payload_monitor.  Every incoming packet is forwarded to the
// monitor, which owns all of the counting / decoding / display logic.  Command
// and reply traffic is forwarded as well so it shows up in the type table.
//
#include "test_handler.h"
#include "monitor.h"

void receive(uint8_t type, void * data, uint16_t size, const void * parameter)
{
	monitorPacket(type, data, size);
}

uint8_t receiveCommand(uint8_t type, void * data, uint16_t size, const void * parameter)
{
	monitorPacket(type, data, size);
	return false;  // monitor only, do not consume
}

void receiveReply(uint8_t type, void * data, uint16_t size, bool ack, const void * parameter)
{
	monitorPacket(type, data, size);
}

bool publish(uint8_t type, uint8_t param)
{
	return true;
}
