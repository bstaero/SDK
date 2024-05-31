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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>

#include "test.h"
#include "main.h"
#include "structs.h"

#include "debug.h"
#include "flight_plan.h"

// variables
bool show_telemetry = false;

// functional definitions
bool sendPayloadData(uint8_t * data, uint8_t size);

// packet for transmision
Packet              tx_packet;


#define CMD_BUF_SIZE 8
Packet cmd_buf[CMD_BUF_SIZE];
uint8_t cmd_buf_start = 0;
uint8_t cmd_buf_end = 0;

#define PKT_BUF_SIZE 8
Packet pkt_buf[PKT_BUF_SIZE];
uint8_t pkt_buf_start = 0;
uint8_t pkt_buf_end = 0;

extern CommunicationsInterface * comm_interface;

extern TelemetryOrientation_t telemetry_orientation;
extern TelemetryPosition_t    telemetry_position;
extern TelemetryPressure_t    telemetry_pressure;
extern TelemetrySystem_t      telemetry_system;
extern TelemetryControl_t     telemetry_control;

extern UserPayload_t          rx_payload;
UserPayload_t                 tx_payload;

FlightPlanMap_t flight_plan_map;
FlightPlan flight_plan;

// for command line (terminal) input
struct termios initial_settings, new_settings;

void printTestHelp() {
	printf("Keys:\n");
	printf("  c   : Send payload control data\n");
	printf("  t   : Toggle Telemetry Display\n");
	printf("\n");
	printf("  p   : print this help\n");
}

bool inputAvailable()  
{
	// check for input on terminal
	struct timeval tv;
	fd_set fds;
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds);
	select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);

	return (FD_ISSET(0, &fds));
}


void initializeTest() {

	// terminal settings to get input
	tcgetattr(0,&initial_settings);

	new_settings = initial_settings;
	new_settings.c_lflag &= ~ICANON;
	new_settings.c_lflag &= ~ECHO;
	new_settings.c_lflag &= ~ISIG;
	new_settings.c_cc[VMIN] = 0;
	new_settings.c_cc[VTIME] = 0;

	tcsetattr(0, TCSANOW, &new_settings);
}

void updateTest() {
	char input; 

	Command_t command;

	uint8_t num_points = 0;
	Waypoint_t temp_waypoint;
	Waypoint_t temp_waypoints[MAX_WAYPOINTS];


	if( inputAvailable() ) {
		input = getchar();

		if(input > 0) {
			switch(input) {
				case 'c':
					printf("Sending payload data\n");

					uint8_t data[128];
					sprintf((char*)data,"This is a test %07.01f\n\r",getElapsedTime());
					sendPayloadData(data,strlen((char*)data));
					break;

				case 't':
					show_telemetry = !show_telemetry;
					break;

				case 'p':
					printTestHelp();
					break;

				case 3: // <CTRL-C> 
					// allow flowthrough
				case 'q':
					printf("Keyboard caught exit signal ...\n");
					running = false;
					break;

				default:
					break;
			}
			input = 0;
		} else {
			clearerr(stdin);
		}
	}

	//if(show_telemetry)
}

void exitTest() {
	tcsetattr(0, TCSANOW, &initial_settings);
}

bool sendPayloadData(uint8_t * data, uint8_t size) {
	uint8_t ptr = 0;

	while(ptr < size) {
		if(size-ptr > 64) tx_payload.size = 64;
		else tx_payload.size = size-ptr;

		memcpy(tx_payload.buffer,&(data[ptr]),tx_payload.size);

		comm_handler->send(PAYLOAD_DATA_CHANNEL_0, (uint8_t *)&tx_payload, sizeof(UserPayload_t), NULL);

		ptr += tx_payload.size;
		tx_payload.size = 0;
		for(uint8_t i=0; i<64; i++) tx_payload.buffer[i] = 0;
	}

	return true;
}

uint16_t commConstruct(uint8_t type, PacketAction_t action, void * data, uint16_t size, const void * parameter, bool uses_address, Packet * packet) { 
	packet->clear();

	if(uses_address) {
		packet->setAddressing(true);
		packet->setFromAddress(ALL_NODES);
		packet->setToAddress(ALL_NODES); // FIXME - should find real address
	} else {
		packet->setAddressing(false);
	}

	packet->setType(type);
	packet->setAction(action);
	packet->setData((uint8_t *)data, size);

	return 0;
}

uint16_t commWrite(uint8_t type, PacketAction_t action, void * data, uint16_t size, const void * parameter) {
	uint16_t tx_size = 0, retval = 0;

	if((type&0xF0) != 0x60 && (type <= 0xE8 || type >= 0xEF) && type != PAYLOAD_S0_SENSORS) {
		if((cmd_buf_end + 1) % CMD_BUF_SIZE == cmd_buf_start) {
			pmesg(VERBOSE_ERROR,"Command Buffer Overflow!\n");
			retval = 0;
		} else {
			commConstruct(type, action, data, size, parameter, true, 
					&cmd_buf[cmd_buf_end]);
			retval = cmd_buf[cmd_buf_end].getSize();

			cmd_buf_end =  (cmd_buf_end + 1) % CMD_BUF_SIZE;
		}

	} else {

		if((pkt_buf_end + 1) % PKT_BUF_SIZE == pkt_buf_start) {
			pmesg(VERBOSE_ERROR,"Packet Buffer Overflow!\n");
			retval = 0;
		} else {
			commConstruct(type, action, data, size, parameter, true, 
					&pkt_buf[pkt_buf_end]);
			retval = pkt_buf[pkt_buf_end].getSize();

			pkt_buf_end =  (pkt_buf_end + 1) % PKT_BUF_SIZE;
		}
	}

	while(cmd_buf_start != cmd_buf_end) {
		tx_size = comm_interface->write(cmd_buf[cmd_buf_start].getPacket(), cmd_buf[cmd_buf_start].getSize(), 0x5300);
		if(tx_size == cmd_buf[cmd_buf_start].getSize()) {
			cmd_buf_start = (cmd_buf_start + 1) % CMD_BUF_SIZE;
		} else {
			return retval;
		}
	}

	while(pkt_buf_start != pkt_buf_end) {
		tx_size = comm_interface->write(pkt_buf[pkt_buf_start].getPacket(), pkt_buf[pkt_buf_start].getSize(), 0x5300);
		if(tx_size == pkt_buf[pkt_buf_start].getSize()) {
			pkt_buf_start = (pkt_buf_start + 1) % PKT_BUF_SIZE;
		} else {
			return retval;
		}
	}

	return retval;
}
