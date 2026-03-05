/*=+--+=#=+--         SwiftCore Flight Management Software        --+=#=+--+=#*\
|               Copyright (C) 2012 Black Swift Technologies LLC.               |
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
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include "comm_interface.h"

#include "debug.h"

CommunicationsInterface::CommunicationsInterface() {

	pmesg(VERBOSE_ALLOC, "CommunicationsInterface::CommunicationsInterface()\n");

	connected = false;
	comm_type = CommunicationsInterface::UNKNOWN;
}

bool CommunicationsInterface::initialize(const char * param1, const char * param2, const char * param3) {
	if(param1 != NULL) strncpy(&param[0][0],param1,MAX_PARAM_SIZE);
	if(param2 != NULL) strncpy(&param[1][0],param2,MAX_PARAM_SIZE);
	if(param3 != NULL) strncpy(&param[2][0],param3,MAX_PARAM_SIZE);
	return true;
}

bool CommunicationsInterface::open(void) { return true; }
bool CommunicationsInterface::close(void) { return true; }

int16_t CommunicationsInterface::read(uint8_t * buffer, uint16_t size, uint64_t *addr) {
	if(addr) addr = 0;
	return this->read(buffer, size);
	
}

int16_t CommunicationsInterface::write(uint8_t * buffer, uint16_t size, uint64_t addr) {
	return this->write(buffer, size);
}

bool CommunicationsInterface::isConnected() {
	return connected;
}
