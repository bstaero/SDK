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

#ifndef _BST_PROTOCOL_H_
#define _BST_PROTOCOL_H_

#include "comm_protocol.h"
#include "bst_module.h"
#include "bst_packet.h"

#include <queue>

#define PACKET_BUFFER_SIZE 8

class BSTCommunicationsModule; // FIXME -- shouldn't need forward declaration if written correctly

class BSTProtocol : public CommunicationsProtocol {
	public:
		BSTProtocol();

		uint16_t update();

		void registerModule(BSTCommunicationsModule * a_module);

		void send(uint8_t type, uint8_t * data, uint16_t size, const void * parameter);
		void sendCommand(uint8_t type, uint8_t * data, uint16_t size, const void * parameter);
		void request(uint8_t type, uint8_t parameter);

		uint8_t write(uint8_t type, uint8_t action, void * data, uint16_t size, const void * parameter);

		void setAddressing(bool on_off);

		uint32_t getLastAddress(void);

	private:
		float last_tx;

		std::queue<Packet> rx_queue;
		std::queue<Packet> tx_queue;
		std::queue<Packet> tx_priority_queue;

		Packet rx_packet;
		Packet tx_packet;

		Packet temp_packet;

		BSTCommunicationsModule * modules[COMM_PROTOCOL_MAX_MODULES];
		uint8_t num_modules;

		void parseData(uint8_t byte);

		bool uses_address;

		uint32_t last_address;

		uint8_t last_request;


};

#endif
