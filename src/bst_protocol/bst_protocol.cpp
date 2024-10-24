#include "bst_protocol.h"
//#include "uart.h"

#include "debug.h"

// FIXME -- remove #define
#ifdef IMPLEMENTATION_firmware
#include "firmware.h"
#endif

extern SystemInitialize_t system_initialize;

extern "C" {
  float getElapsedTime(); // defined elsewhere
}

BSTProtocol::BSTProtocol() : CommunicationsProtocol() {
	pmesg(VERBOSE_ALLOC, "BSTProtocol::BSTProtocol()\n");
	uses_address = true;

	for(uint8_t i=0u; i<COMM_PROTOCOL_MAX_MODULES; i++)
		modules[i] = NULL;
	num_modules = 0;

	last_tx = getElapsedTime();

	last_request = INVALID_PACKET;
}

void BSTProtocol::registerModule(BSTCommunicationsModule * a_module) {
	if(a_module != NULL && num_modules < COMM_PROTOCOL_MAX_MODULES) {
		a_module -> setProtocol(this);
		modules[num_modules++] = a_module;
	}
}

//FIXME have to do this otherwise it will go into lost comms when asking for parameters or sending flight plans
#if defined NO_DUPLEX_COMMS
extern float last_heartbeat;
#endif

void BSTProtocol::parseData(uint8_t byte) {

	if(rx_queue.size() >= PACKET_BUFFER_SIZE) {
		pmesg(VERBOSE_ERROR,"Receive Buffer Overflow!\n");
		return;
	}

	if(rx_packet.isValid(byte)) {
//FIXME have to do this otherwise it will go into lost comms when asking for parameters or sending flight plans
#ifdef NO_DUPLEX_COMMS
		if(rx_packet.getType() != TELEMETRY_GCS_LOCATION)
			last_heartbeat = getElapsedTime();
#endif
		rx_queue.push(rx_packet);
		rx_packet.clear();
	}
}

#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
static float RADIO_TIMEOUT = 0.25;
#endif

#if defined (NO_DUPLEX_COMMS)
static float CMD_TIMEOUT = 3.0;
static float last_cmd_rx = -1.0;
#endif

uint16_t BSTProtocol::update() {
	uint16_t n = CommunicationsProtocol::update();

	for(uint8_t i=0u; i<num_modules; i++) {
		if( modules[i] ) {
			modules[i]->update();
		}
	}

	if(rx_queue.size() > 0) {
		temp_packet = rx_queue.front();
		last_address = temp_packet.getFromAddress();
		rx_queue.pop();

#if defined (NO_DUPLEX_COMMS)
		if((temp_packet.getType() & 0xF0) != 0x60 &&
				temp_packet.getType() != TELEMETRY_DEPLOYMENT_TUBE &&
				temp_packet.getType() != PAYLOAD_S0_SENSORS ) {

			if(temp_packet.getType() >= PAYLOAD_DATA_CHANNEL_0 && temp_packet.getType() <= PAYLOAD_DATA_CHANNEL_7) {
				if(temp_packet.getAction() != PKT_ACTION_STATUS) {
					last_cmd_rx = getElapsedTime();
					last_request = temp_packet.getType();
#if defined (NO_DUPLEX_COMMS)
					if(temp_packet.getType() == FLIGHT_PLAN ||
						 temp_packet.getType() == FLIGHT_PLAN_MAP ||
						 temp_packet.getType() == LAST_MAPPING_WAYPOINT ||
						 temp_packet.getType() == FLIGHT_PLAN_WAYPOINT ||
						 temp_packet.getType() == SYSTEM_INITIALIZE) {
						CMD_TIMEOUT = 3.0;
#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
						RADIO_TIMEOUT = 0.25;
#endif
					} else {
						CMD_TIMEOUT = 1.0;
#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
						RADIO_TIMEOUT = 0.02;
#endif
					}
#endif
				} 
			} else {
				last_cmd_rx = getElapsedTime();
				last_request = temp_packet.getType();
#if defined (NO_DUPLEX_COMMS)
				if(temp_packet.getType() == FLIGHT_PLAN ||
						temp_packet.getType() == FLIGHT_PLAN_MAP ||
						temp_packet.getType() == LAST_MAPPING_WAYPOINT ||
						temp_packet.getType() == FLIGHT_PLAN_WAYPOINT ||
						temp_packet.getType() == SYSTEM_INITIALIZE) {
					CMD_TIMEOUT = 3.0;
#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
					RADIO_TIMEOUT = 0.25;
#endif
				} else {
					CMD_TIMEOUT = 1.0;
#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
					RADIO_TIMEOUT = 0.02;
#endif
				}
#endif
			}
		}
#endif

		for(uint8_t i =0; i< num_modules; i++) { if(modules[i] && modules[i]->handles(temp_packet.getType())) {
				modules[i]->parse(temp_packet.getType(), temp_packet.getAction(), (uint8_t *)temp_packet.getDataPtr(), temp_packet.getDataSize());
			}
		}
	}

#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
	if(getElapsedTime() - last_tx > RADIO_TIMEOUT) {
#endif
		if(tx_priority_queue.size() > 0) {
			temp_packet = tx_priority_queue.front();
			if(CommunicationsProtocol::write(temp_packet.getPacket(), temp_packet.getSize()) == temp_packet.getSize()) {
				tx_priority_queue.pop();
#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
				last_tx = getElapsedTime();
#endif
			}
		} else {
#if defined (NO_DUPLEX_COMMS)
			if(last_cmd_rx >= 0 && getElapsedTime() - last_cmd_rx < CMD_TIMEOUT) {
				return 0;
			}
#endif
			if(tx_queue.size() > 0) {
				temp_packet = tx_queue.front();
				if(CommunicationsProtocol::write(temp_packet.getPacket(), temp_packet.getSize()) == temp_packet.getSize()) {
					tx_queue.pop();
#if defined (NO_DUPLEX_COMMS)
					if(temp_packet.getType() == FLIGHT_PLAN ||
							temp_packet.getType() == FLIGHT_PLAN_MAP ||
							temp_packet.getType() == LAST_MAPPING_WAYPOINT ||
							temp_packet.getType() == FLIGHT_PLAN_WAYPOINT ||
							temp_packet.getType() == SYSTEM_INITIALIZE) {
						//last_cmd_rx = getElapsedTime();
						CMD_TIMEOUT = 3.0;
#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
						RADIO_TIMEOUT = 0.25;
#endif
					} else {
						CMD_TIMEOUT = 1.0;
#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
						RADIO_TIMEOUT = 0.02;
#endif
					}
#endif
#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
					last_tx = getElapsedTime();
#endif
				}
			}
		}
#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
	}
#endif

	//FIXME -- need lost comm
	/*float now = getElapsedTime();
	if(mission_parameters.comm.seconds > 0 && !system_status.lost_comm) {
		if(now - last_heartbeat >= mission_parameters.comm.seconds) {
			pmesg(VERBOSE_ERROR, "!!!!! LOST COMM !!!!\n");
			system_status.lost_comm = true;
		}
	} else {
		if(mission_parameters.comm.seconds == 0 && system_status.lost_comm) 
			system_status.lost_comm = false; // TODO -- should we allow user to turn off lost comm?
	}*/

	return n;
}

void BSTProtocol::send(uint8_t type, uint8_t * data, uint16_t size, const void * parameter) {
	for(uint8_t i=0u; i< num_modules; i++) {
		if(modules[i]->handles(type))
			modules[i]->send(type, data, size, parameter);
	}
}

void BSTProtocol::sendCommand(uint8_t type, uint8_t * data, uint16_t size, const void * parameter) {
	for(uint8_t i=0u; i< num_modules; i++) {
		if(modules[i]->handles(type))
			modules[i]->sendCommand(type, data, size, parameter);
	}
}

void BSTProtocol::request(uint8_t type, uint8_t parameter) {
	for(uint8_t i=0u; i< num_modules; i++) {
		if(modules[i]->handles(type))
			modules[i]->request(type, parameter);
	}
}

uint8_t BSTProtocol::write(uint8_t type, uint8_t action, void * data, uint16_t size, const void * parameter) {
	uint8_t temp_request = last_request;
	last_request = INVALID_PACKET;

	tx_packet.clear();

	if(uses_address) {
		tx_packet.setAddressing(true);
		tx_packet.setFromAddress(system_initialize.serial_num);
		//tx_packet.setFromAddress(ALL_NODES);
		tx_packet.setToAddress(ALL_NODES); // FIXME - should find real address
		//tx_packet.setToAddress(ALL_UAVS); // FIXME - should find real address
	} else {
		tx_packet.setAddressing(false);
	}

	tx_packet.setType(type);
	tx_packet.setAction((PacketAction_t)action);
	tx_packet.setData((uint8_t *)data, size);

	if((type & 0xF0) != 0x60 &&
			type != TELEMETRY_DEPLOYMENT_TUBE &&
			type != PAYLOAD_S0_SENSORS ) {

		if(type >= PAYLOAD_DATA_CHANNEL_0 && type <= PAYLOAD_DATA_CHANNEL_7) {
			if(action != PKT_ACTION_STATUS || temp_request == type) {
				if(tx_priority_queue.size() > PACKET_BUFFER_SIZE) {
					pmesg(VERBOSE_ERROR,"Prioity Transmit Command Buffer Overflow!\n");
					return 0;
				}

				tx_priority_queue.push(tx_packet);
			} else {
				if(tx_queue.size() > PACKET_BUFFER_SIZE) {
					pmesg(VERBOSE_ERROR,"Transmit Command Buffer Overflow!\n");
					return 0;
				}

				tx_queue.push(tx_packet);
			}
		}  else {
			if(tx_priority_queue.size() > PACKET_BUFFER_SIZE) {
				pmesg(VERBOSE_ERROR,"Prioity Transmit Command Buffer Overflow!\n");
				return 0;
			}

			tx_priority_queue.push(tx_packet);
		}
	} else {
		if(tx_queue.size() > PACKET_BUFFER_SIZE) {
			pmesg(VERBOSE_ERROR,"Transmit Command Buffer Overflow!\n");
			return 0;
		}

		tx_queue.push(tx_packet);
	}

#ifdef IMPLEMENTATION_firmware
	if((type&0xF0) != 0x60) // Don't include telemetry packets
		writeLogFile(type, (PacketAction_t)action, data, size, parameter);
#endif

	return 1;
}

void BSTProtocol::setAddressing(bool on_off) {
	uses_address = on_off;

	rx_packet.setAddressing(on_off);
	tx_packet.setAddressing(on_off);
}

uint32_t BSTProtocol::getLastAddress() {
	return last_address;
}
