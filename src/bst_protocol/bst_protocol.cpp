#include "bst_protocol.h"
//#include "uart.h"

#include "debug.h"

// FIXME -- remove #define
#ifdef IMPLEMENTATION_firmware
#include "firmware.h"
#else
uint16_t commWrite(uint8_t type, PacketAction_t action, 
		void * data, uint16_t size, const void * parameter);
#endif

extern SystemInitialize_t system_initialize;

BSTProtocol::BSTProtocol() : CommunicationsProtocol() {
	pmesg(VERBOSE_ALLOC, "BSTProtocol::BSTProtocol()\n");
	uses_address = true;

	for(uint8_t i=0u; i<COMM_PROTOCOL_MAX_MODULES; i++)
		modules[i] = NULL;
	num_modules = 0;
}

void BSTProtocol::registerModule(BSTCommunicationsModule * a_module) {
	if(a_module != NULL && num_modules < COMM_PROTOCOL_MAX_MODULES) {
		a_module -> setProtocol(this);
		modules[num_modules++] = a_module;
	}
}

void BSTProtocol::parseData(uint8_t byte) {
	if(rx_packet.isValid(byte)) {
		rx_queue.push(rx_packet);
		rx_packet.clear();
	}
}

uint16_t BSTProtocol::update() {
	uint16_t n = CommunicationsProtocol::update();

	for(uint8_t i=0u; i<num_modules; i++) {
		if( modules[i] ) {
			modules[i]->update();
		}
	}

	uint8_t cnt = 0;
	while(rx_queue.size() > 0 && ++cnt <= 3) {
		temp_packet = rx_queue.front();
		last_address = temp_packet.getFromAddress();
		rx_queue.pop();
		for(uint8_t i =0; i< num_modules; i++) { if(modules[i] && modules[i]->handles(temp_packet.getType())) {
				modules[i]->parse(temp_packet.getType(), temp_packet.getAction(), (uint8_t *)temp_packet.getDataPtr(), temp_packet.getDataSize());
			}
		}
	}

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
#if 0
		tx_packet.clear();

	if(uses_address) {
		tx_packet.setAddressing(true);
		tx_packet.setFromAddress(system_initialize.serial_num);
		//tx_packet.setFromAddress(0xFF);
		tx_packet.setToAddress(ALL_NODES); // FIXME - should find real address
		//tx_packet.setToAddress(ALL_UAVS); // FIXME - should find real address
	} else {
		tx_packet.setAddressing(false);
	}

	tx_packet.setType(type);
	tx_packet.setAction((PacketAction_t)action);
	tx_packet.setData((uint8_t *)data, size);

// FIXME -- need a better way to implement logging all of the packets
#ifdef IMPLEMENTATION_firmware
	if((type&0xF0) != 0x60) // Don't include telemetry packets
		writeLogFile(type, (PacketAction_t)action, data, size, parameter);
#endif

	//return CommunicationsProtocol::write(tx_packet.getPacket(), tx_packet.getSize(), 0x5300);
	return CommunicationsProtocol::write(tx_packet.getPacket(), tx_packet.getSize());
#else
	return commWrite(type, (PacketAction_t)action, data, size, parameter);
#endif
}

void BSTProtocol::setAddressing(bool on_off) {
	uses_address = on_off;

	rx_packet.setAddressing(on_off);
	tx_packet.setAddressing(on_off);
}

uint32_t BSTProtocol::getLastAddress() {
	return last_address;
}
