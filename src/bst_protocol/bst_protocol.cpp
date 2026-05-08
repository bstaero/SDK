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
	default_to_address = ALL_NODES;

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

// P400 NB Transparent mode timeout tuning (S128=0, S103=3 → 4800 bps OTA):
// At 4800 bps, a 50-byte packet takes ~83ms over the air, 100 bytes ~167ms.
// S113=5 radio-level retransmissions each add a full packet time.
// S136=1 (RX priority) means radio is effectively half-duplex.
// Values must account for: packet TX time + turnaround + possible retransmissions.
#if defined(RADIO_P400)
#define CMD_TIMEOUT_FP     5.0   // Flight plan operations (s) — FP packets are large
#define CMD_TIMEOUT_STD    2.0   // Normal commands (s) — allows ~3 retransmissions at 4800 bps
#define RADIO_TIMEOUT_FP   0.35  // TX spacing during FP exchange (s) — 167ms TX + turnaround
#define RADIO_TIMEOUT_STD  0.20  // TX spacing during normal commands (s) — 83ms TX + margin
#define RADIO_TIMEOUT_INIT 0.25  // Default TX spacing (s)
#else
#define CMD_TIMEOUT_FP     3.0
#define CMD_TIMEOUT_STD    1.0
#define RADIO_TIMEOUT_FP   0.25
#define RADIO_TIMEOUT_STD  0.02
#define RADIO_TIMEOUT_INIT 0.25
#endif

#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
static float RADIO_TIMEOUT = RADIO_TIMEOUT_INIT;
#endif

#if defined (NO_DUPLEX_COMMS)
static float CMD_TIMEOUT = CMD_TIMEOUT_FP;
static float last_cmd_rx = -1.0;
#endif

uint16_t BSTProtocol::update() {
	uint16_t n = CommunicationsProtocol::update();

	for(uint8_t i=0u; i<num_modules; i++) {
		if( modules[i] ) {
			modules[i]->update();
		}
	}

	// Process up to 3 rx packets per update to avoid blocking the main loop.
	// Even socket builds may sit behind a radio link or simulate real timing.
	{ uint8_t rx_cnt = 0;
	while(rx_queue.size() > 0 && ++rx_cnt <= 3) {
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
						CMD_TIMEOUT = CMD_TIMEOUT_FP;
#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
						RADIO_TIMEOUT = RADIO_TIMEOUT_FP;
#endif
					} else {
						CMD_TIMEOUT = CMD_TIMEOUT_STD;
#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
						RADIO_TIMEOUT = RADIO_TIMEOUT_STD;
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
					CMD_TIMEOUT = CMD_TIMEOUT_FP;
#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
					RADIO_TIMEOUT = RADIO_TIMEOUT_FP;
#endif
				} else {
					CMD_TIMEOUT = CMD_TIMEOUT_STD;
#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
					RADIO_TIMEOUT = RADIO_TIMEOUT_STD;
#endif
				}
#endif
			}
		}
#endif

		// filter packets not addressed to this node (matches exact address,
		// ALL_UAVS, ALL_NODES, and ONE_HOP via Packet::isToID mask logic)
		if(uses_address && !temp_packet.isToID(system_initialize.serial_num)) {
			continue;
		}

		// track last non-telemetry request for response addressing
		if((temp_packet.getType() & 0xF0) != 0x60 &&
				temp_packet.getAction() != PKT_ACTION_STATUS) {
			last_request = temp_packet.getType();
		}

		for(uint8_t i =0; i< num_modules; i++) { if(modules[i] && modules[i]->handles(temp_packet.getType())) {
				modules[i]->parse(temp_packet.getType(), temp_packet.getAction(), (uint8_t *)temp_packet.getDataPtr(), temp_packet.getDataSize());
			}
		}
	}
	} // rx_cnt scope

#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
	if(getElapsedTime() - last_tx > RADIO_TIMEOUT) {
#endif
		// Drain TX queues: priority first, then regular.
		// For radio/serial (LOW_BANDWIDTH), send one packet per update to stay bounded.
		// For socket builds (SWIL), drain up to 8 to handle TCP initialization bursts.
#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
		uint8_t max_tx = 1;
#else
		uint8_t max_tx = 8;
#endif
		for(uint8_t tx_i = 0; tx_i < max_tx; tx_i++) {
		if(tx_priority_queue.size() > 0) {
			temp_packet = tx_priority_queue.front();
			if(CommunicationsProtocol::write(temp_packet.getPacket(), temp_packet.getSize()) == temp_packet.getSize()) {
				tx_priority_queue.pop();
#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
				last_tx = getElapsedTime();
#endif
			} else break;
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
						CMD_TIMEOUT = CMD_TIMEOUT_FP;
#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
						RADIO_TIMEOUT = RADIO_TIMEOUT_FP;
#endif
					} else {
						CMD_TIMEOUT = CMD_TIMEOUT_STD;
#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
						RADIO_TIMEOUT = RADIO_TIMEOUT_STD;
#endif
					}
#endif
#if defined LOW_BANDWIDTH || defined SERIAL_COMMS
					last_tx = getElapsedTime();
#endif
				} else break;
			} else break;
		}
		} // for tx_i
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

		// responses to a request go back to the requester's address;
		// unsolicited telemetry uses the default broadcast address
		if(temp_request != INVALID_PACKET && last_address != NO_ID) {
			tx_packet.setToAddress(last_address);
		} else {
			tx_packet.setToAddress(default_to_address);
		}
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
					pmesg(VERBOSE_ERROR,"Transmit Buffer Overflow! type=%u qsize=%u\n", tx_packet.getType(), (unsigned)tx_queue.size());
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
	// Log non-telemetry command responses (ACK/NACK/REQUEST) for protocol debugging.
	if((type&0xF0) != 0x60 && action != PKT_ACTION_STATUS)
		writeLogFile(type, (PacketAction_t)action, data, size, parameter);
#endif

	return 1;
}

void BSTProtocol::setAddressing(bool on_off) {
	uses_address = on_off;

	rx_packet.setAddressing(on_off);
	tx_packet.setAddressing(on_off);
}

void BSTProtocol::setDefaultToAddress(uint32_t addr) {
	default_to_address = addr;
}

uint32_t BSTProtocol::getLastAddress() {
	return last_address;
}
