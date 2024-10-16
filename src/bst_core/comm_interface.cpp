/*=+--+=#=+--            SwiftPilot Autopilot Software            --+=#=+--+=#*\
|               Copyright (C) 2012 Black Swift Technologies LLC.               |
|                             All Rights Reserved.                             |

     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License version 2 as
     published by the Free Software Foundation.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                  Jack Elston                                   
|                          elstonj@blackswifttech.com                          |
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
