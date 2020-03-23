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

static CommunicationsInterface * can_interface = NULL;

static uint8_t can_rx_buffer[CAN_BUF_SIZE];
static uint8_t can_tx_buffer[CAN_BUF_SIZE];

static Packet tx_packet;
static Packet rx_packet;

void setupSimulatedCAN(CommunicationsInterface * interface) {
	can_interface = interface;
	can_interface->open();

	rx_packet.setAddressing(false);
	tx_packet.setAddressing(false);

	rx_packet.clear();
	tx_packet.clear();
}

#ifdef __cplusplus
  extern "C" {
#endif

uint8_t simulatedCANRead(void) {
	if(can_interface == NULL) return 0;

	uint16_t num_read = can_interface->read(can_rx_buffer,CAN_BUF_SIZE);
	if(num_read > 0) {

		uint16_t byte = 0;
		while(byte < num_read) {
			if(rx_packet.isValid(can_rx_buffer[byte++])) {
				if(rx_packet.getType() == HWIL_CAN) {

					uint32_t id;
					uint8_t size;
					uint16_t ptr = 0;

					const uint8_t * data_ptr = rx_packet.getDataPtr();

					while(ptr < num_read) {
						memcpy(&id,(void *)(data_ptr + ptr),4); ptr += 4;
						memcpy(&size,(void *)(data_ptr + ptr),1); ptr += 1;
						BRIDGE_Arbiter(id,(void *)(data_ptr + ptr), size); ptr += size;
					}

					rx_packet.clear();

				}
			}
		}

	}

	return num_read;
}

uint8_t simulatedCANWrite(uint8_t p, uint32_t id, void *data, uint8_t size) {
	if(can_interface == NULL) return 0;
	if(size + 5 > CAN_BUF_SIZE) return 0;

	uint8_t ptr = 0;
	memcpy(can_tx_buffer + ptr,&id,4); ptr += 4;
	memcpy(can_tx_buffer + ptr,&size,1); ptr += 1;
	memcpy(can_tx_buffer + ptr,data,size); ptr += size;

	tx_packet.clear();
	tx_packet.setType(HWIL_CAN);
	tx_packet.setAction(PKT_ACTION_STATUS);
	tx_packet.setData(can_tx_buffer, ptr);

	can_interface->write(tx_packet.getPacket(),tx_packet.getSize());

	return size;
}

#ifdef __cplusplus
}
#endif
