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

#ifndef _COMM_PROTOCOL_H_
#define _COMM_PROTOCOL_H_

#define COMM_PROTOCOL_BUFF_SIZE 256
#define COMM_PROTOCOL_MAX_MODULES 10

#include "comm_interface.h"

class CommunicationsProtocol {
	public:

		CommunicationsProtocol();
		~CommunicationsProtocol();

		virtual uint16_t update();

		virtual void send(uint8_t type, uint8_t * data, uint16_t size, const void * parameter) = 0;
		virtual void sendCommand(uint8_t type, uint8_t * data, uint16_t size, const void * parameter) = 0;
		virtual void request(uint8_t type, uint8_t parameter) = 0;

		virtual void setAddressing(bool on_off) = 0;

		CommunicationsInterface * getInterface() { return interface; }
		void setInterface(CommunicationsInterface * an_interface) { interface = an_interface; }


		inline float getLastCommunication() {return last_communication;}

	protected:
		float last_communication;

		virtual void parseData(uint8_t byte) = 0;

		CommunicationsInterface * interface;

		uint16_t read(uint8_t * buffer, uint16_t size);
		uint16_t write(uint8_t * buffer, uint16_t size);

		uint8_t __attribute__((aligned)) data[COMM_PROTOCOL_BUFF_SIZE];
};

#endif
