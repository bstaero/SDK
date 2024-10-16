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

#ifndef _COMM_INTERFACE_H
#define _COMM_INTERFACE_H

#include <string.h>
#include <inttypes.h>
#include <stdio.h>

#define MAX_PARAM_SIZE 32

class CommunicationsInterface {
	public:

		typedef enum CommTypes {SOCKET_UDP,SOCKET_TCP,SERIAL,XBEE,XTEND,XETAWAVE,U_HARD,LOCAL_FILE,UNKNOWN} CommType_t;

		CommunicationsInterface();
		virtual ~CommunicationsInterface() { }

		virtual bool initialize(const char * = NULL, const char * = NULL, const char * = NULL);
		virtual bool open(void);
		virtual bool close(void);

		virtual int16_t read(uint8_t * buffer, uint16_t size) = 0;
		virtual int16_t write(uint8_t * buffer, uint16_t size) = 0;

		virtual int16_t read(uint8_t * buffer, uint16_t size, uint64_t *addr);
		virtual int16_t write(uint8_t * buffer, uint16_t size, uint64_t addr);

		virtual bool isConnected();

		virtual uint16_t getRxBytes() = 0;
		virtual uint16_t getTxBytes() = 0;

		CommType_t getType() {return comm_type;}

		char * getParameter(uint8_t num) {return &param[num][0];}

	protected:
		bool connected;
		CommType_t comm_type;
		
		char param[3][MAX_PARAM_SIZE];
};

#endif
