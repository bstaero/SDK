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

#include <stdio.h>
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#if !defined __APPLE__ && !defined __OPENWRT__
  #include <error.h>
#endif
#if defined __APPLE__
  #include <mach/clock.h>
  #include <mach/mach.h>
#endif
#include <errno.h>
#include <time.h>
#include <math.h>

#include "bst_interface.h"
#include "debug.h"

#define CONNECTION_TIMEOUT 1.f // [s]

BSTInterface::BSTInterface () : CommunicationsInterface() {

	pmesg(VERBOSE_ALLOC, "BSTInterface::BSTInterface()\n");

	this->fd = -1;

	rx_bytes = 0;
	tx_bytes = 0;

	last_connection_attempt = 0.0;
}


#ifdef __APPLE__
  #include <mach/mach_time.h> // system time
#endif

float getCurrentTime() {
	float current_time = 0;

#ifdef __APPLE__
	uint64_t now = mach_absolute_time();
	float conversion  = 0.0;
	mach_timebase_info_data_t info;
	kern_return_t err = mach_timebase_info( &info );
	if( err == 0  )
		conversion = 1e-9 * (float) info.numer / (float) info.denom;
	current_time = conversion * (float) now;
#else
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	current_time = (float)now.tv_sec + (float)now.tv_nsec / 10e8;
#endif
	return current_time;
}


bool BSTInterface::checkConnected() {
	if (!connected || this->fd < 0) {

		// make sure its closed
		this->close();

		float current_time = getCurrentTime();

		if((current_time - last_connection_attempt) < CONNECTION_TIMEOUT)
			return false;

		last_connection_attempt = current_time;

		// now try to open
		if(!this->open()) {
			connected = false;
		} else
			connected = true;
	}

	return connected;
}

int16_t BSTInterface::read(uint8_t * buf, uint16_t buf_size) {
	if(!checkConnected()) return 0;

	int n = ::read(this->fd,buf,buf_size);
	if (n < 0) {
		if(errno != EAGAIN && errno != EWOULDBLOCK) {
			pmesg(VERBOSE_ERROR, "BSTInterface::ERROR - failed on read: %s\n", strerror(errno));
			this->close();
		}
		n = 0;
	} else if (n != 0) {
		rx_bytes += n;
	}

	return n;
}

int16_t BSTInterface::write(uint8_t * buf, uint16_t size) {
	if(!checkConnected()) return 0;

	int n = ::write(this->fd,buf,size);
	if (n < 0) {
		if(errno != EAGAIN && errno != EWOULDBLOCK)
			connected = false;
		n = 0;
	}	else {
		tx_bytes += n;
	}

	return n;
}

bool BSTInterface::close() {
	if(this->fd >= 0)
		::close(this->fd);

	// make sure we have a consistent state
	this->fd = -1;
	connected = false;

	return true;
}
