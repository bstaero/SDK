#include "comm_protocol.h"

#include "debug.h"

CommunicationsProtocol::CommunicationsProtocol () {
	pmesg(VERBOSE_ALLOC, "CommunicationsProtocol::CommunicationsProtocol()\n");

	last_communication = 0.0f;
}

CommunicationsProtocol::~CommunicationsProtocol () {
	if(interface)
		if(interface->close()) interface = NULL;
}

uint16_t CommunicationsProtocol::update() {
	uint16_t n = 0u;

	if( (n = read(data, COMM_PROTOCOL_BUFF_SIZE)) ) {
		for(uint16_t i=0u; i<n; i++) {
			parseData(data[i]);
		}
	}

	return n;
}

uint16_t CommunicationsProtocol::read(uint8_t * buffer, uint16_t size) {
	if(interface == NULL) return 0u;
	return interface->read(buffer,size);
}

uint16_t CommunicationsProtocol::write(uint8_t * buffer, uint16_t size) {
	if(interface == NULL) return 0u;
	return interface->write(buffer,size);
}
