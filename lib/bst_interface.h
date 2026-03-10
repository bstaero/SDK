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

#ifndef _BST_INTERFACE_H
#define _BST_INTERFACE_H

#include <string.h>

#include "comm_interface.h"

class BSTInterface : public CommunicationsInterface {
	public:
		BSTInterface();
		virtual ~BSTInterface() { }

		virtual bool initialize(const char * param1, const char * param2, const char * param3) = 0;

		virtual bool open() = 0;
		virtual bool close();

		int16_t read(uint8_t * buf, uint16_t buf_size);
		int16_t write(uint8_t * buf, uint16_t size);

		uint16_t getRxBytes() {return rx_bytes;}
		uint16_t getTxBytes() {return tx_bytes;}

	protected:

		bool checkConnected();

		float last_connection_attempt;

		long rx_bytes;
		long tx_bytes;

		volatile int fd;
};

#endif
