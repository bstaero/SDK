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
#ifndef _BST_MODULE_H_
#define _BST_MODULE_H_

#include "bst_packet.h"
#include "bst_protocol.h"

#define MAX_DATA_TYPES INVALID_PACKET

class BSTProtocol; // FIXME -- shouldn't need forward declaration if written correctly

class BSTCommunicationsModule {
	public:
		BSTCommunicationsModule();
		~BSTCommunicationsModule();

		virtual void update();


		bool handles(uint8_t type);

		virtual void send(uint8_t type, uint8_t * data, uint16_t size, const void * parameter);
		virtual void sendCommand(uint8_t type, uint8_t * data, uint16_t size, const void * parameter);
		virtual void request(uint8_t type, uint8_t parameter);

		virtual void parse(uint8_t type, uint8_t action, uint8_t * data, uint16_t size);


		void registerReceive(void (*)(uint8_t, void *, uint16_t, const void *));

		void registerReceiveCommand(uint8_t (*)(uint8_t, void *, uint16_t, const void *));
		void registerReceiveReply(void (*)(uint8_t, void *, uint16_t, bool, const void *));
		void registerPublish(bool (*)(uint8_t, uint8_t));

		void setProtocol(BSTProtocol *);


	protected:

		void (* receive_function)(uint8_t, void *, uint16_t, const void *);

		uint8_t (* receiveCommand_function)(uint8_t, void *, uint16_t, const void *);
		void (* receiveReply_function)(uint8_t, void *, uint16_t, bool, const void *);

		bool (* publish_function)(uint8_t, uint8_t);

		typedef struct _DataType_t {
			uint16_t size;
			bool command;
			bool request;
			_DataType_t() {
				size = 0;
				command = false;
				request = false;
			}
		} DataType_t;

		uint8_t max_num_data_types;
		uint8_t num_data_types;
		DataType_t * data_types;
		uint8_t data_types_lut[MAX_DATA_TYPES]; // lookup table

		uint8_t local_data[BST_MAX_PACKET_SIZE];

		BSTProtocol * parent;

		void registerDataType(uint8_t type, uint16_t size, bool command, bool request);

};

#endif
