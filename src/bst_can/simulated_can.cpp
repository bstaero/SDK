/*=+--+=#=+--            SwiftPilot Autopilot Software            --+=#=+--+=#*\
|               Copyright (C) 2013 Black Swift Technologies LLC.               |
|                             All Rights Reserved.                             |

                                  Jack Elston                                   
|                          elstonj@blackswifttech.com                          |
|                                                                              |
\*=+--+=#=+--                 --+=#=+--+=#=+--                    --+=#=+--+=#*/

/*!
  \file simulated_can.cpp
	\brief Implementation for software in the loop simulation of CAN bus.

  \author Jack Elston
  \date
*/

#include "simulated_can.h"
#include "bst_packet.h"
#include "debug.h"

#define CAN_BUF_SIZE 500
#define CAN_MESSAGE 0x80

#define MAX_CAN_INTERFACES 2

static CommunicationsInterface * can_interface [MAX_CAN_INTERFACES];

static uint8_t num_can_interfaces = 0;

static uint8_t can_rx_buffer[MAX_CAN_INTERFACES][CAN_BUF_SIZE];
static uint8_t can_tx_buffer[MAX_CAN_INTERFACES][CAN_BUF_SIZE];

static Packet tx_packet[MAX_CAN_INTERFACES];
static Packet rx_packet[MAX_CAN_INTERFACES];

void setupSimulatedCAN(CommunicationsInterface * interface) {
	can_interface[num_can_interfaces] = interface;
	can_interface[num_can_interfaces]->open();

	rx_packet[num_can_interfaces].setAddressing(false);
	tx_packet[num_can_interfaces].setAddressing(false);

	rx_packet[num_can_interfaces].clear();
	tx_packet[num_can_interfaces].clear();

	num_can_interfaces ++;
}

#ifdef __cplusplus
  extern "C" {
#endif

uint8_t simulatedCANRead(uint8_t p) {
	if(p == 0 || p > num_can_interfaces) return 0;
	if(can_interface[p-1] == NULL) return 0;

	uint16_t num_read = can_interface[p-1]->read(can_rx_buffer[p-1],CAN_BUF_SIZE);
	if(num_read > 0) {

		uint16_t byte = 0;
		while(byte < num_read) {
			if(rx_packet[p-1].isValid(can_rx_buffer[p-1][byte++])) {
				if(rx_packet[p-1].getType() == HWIL_CAN) {

					uint32_t id;
					uint8_t size;
					uint16_t ptr = 0;

					const uint8_t * data_ptr = rx_packet[p-1].getDataPtr();

					memcpy(&id,(void *)(data_ptr + ptr),4); ptr += 4;
					memcpy(&size,(void *)(data_ptr + ptr),1); ptr += 1;
					BRIDGE_Arbiter(id,(void *)(data_ptr + ptr), size); ptr += size;

					rx_packet[p-1].clear();

				}
			}
		}

	}

	return num_read;
}

uint8_t simulatedCANWrite(uint8_t p, uint32_t id, void *data, uint8_t size) {
	if(p == 0 || p > num_can_interfaces) return 0;
	if(can_interface[p-1] == NULL) return 0;
	if(size + 5 > CAN_BUF_SIZE) return 0;

	uint8_t ptr = 0;
	memcpy(can_tx_buffer[p-1] + ptr,&id,4); ptr += 4;
	memcpy(can_tx_buffer[p-1] + ptr,&size,1); ptr += 1;
	memcpy(can_tx_buffer[p-1] + ptr,data,size); ptr += size;

	tx_packet[p-1].clear();
	tx_packet[p-1].setType(HWIL_CAN);
	tx_packet[p-1].setAction(PKT_ACTION_STATUS);
	tx_packet[p-1].setData(can_tx_buffer[p-1], ptr);

	can_interface[p-1]->write(tx_packet[p-1].getPacket(),tx_packet[p-1].getSize());

	return size;
}

#ifdef __cplusplus
}
#endif
