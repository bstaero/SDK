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

#include "bridge.h"


// variables
volatile bool display_telemetry = false;
volatile bool write_file = false;

bool show_gps = false;
bool show_mag = false;
bool show_dynamic = false;
bool show_static = false;

bool new_gps = false;
bool new_mag = false;
bool new_dynamic = false;
bool new_static = false;

bool print_timing = false;

bool auto_test = false;

extern uint32_t gnss_lla_cnt;
extern uint32_t gnss_utc_cnt;
extern uint32_t gnss_vel_cnt;
extern uint32_t gnss_hs_cnt;

extern uint32_t mag_cnt;

extern uint32_t stat_p_cnt;


#define CMD_BUF_SIZE 8
Packet cmd_buf[CMD_BUF_SIZE];
uint8_t cmd_buf_start = 0;
uint8_t cmd_buf_end = 0;

#define PKT_BUF_SIZE 8
Packet pkt_buf[PKT_BUF_SIZE];
uint8_t pkt_buf_start = 0;
uint8_t pkt_buf_end = 0;

extern CommunicationsInterface * comm_interface;

// functional definitions

// packet for transmision
Packet              tx_packet;

// for command line (terminal) input
struct termios initial_settings, new_settings;

void printTestHelp() {
	printf("Keys:\n");
	printf("  t   : Toggle telemetry display\n");
	printf("\n");
	printf("  T   : take a picture\n");
	printf("  1   : test channel 0\n");
	printf("  2   : test channel 1\n");
	printf("  3   : test channel 2\n");
	printf("  4   : test channel 3\n");
	printf("  5   : test channel 4\n");
	printf("  6   : test channel 5\n");
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
	static char auto_char = '0'; 
	static uint8_t is_triggering = 0;
	static float trigger_time = 0;
	static uint16_t actuators[16];

	if( inputAvailable() || auto_test ) {
		if(auto_test) {
			if(!is_triggering) auto_char++;
			if(auto_char > '6') auto_char = 'q';
			input = auto_char;
		} else  {
			input = getchar();
		}

		if(input > 0) {
			switch(input) {

				case 't':
					display_telemetry? display_telemetry=false: display_telemetry=true;
					break;

				case 'T':
					if(!is_triggering) {
						is_triggering = 15;
						trigger_time = getElapsedTime();
					}
					break;

				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
					if(!is_triggering) {
						printf("Triggerging channel %u\n",input - '0');
						is_triggering = input - '0';
						trigger_time = getElapsedTime();
					}
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

	for(uint8_t i=0; i<16; i++) {
		if(is_triggering && (is_triggering-1) == i) {
			if(i < 6)
				actuators[i] = 1800;
			else
				actuators[i] = 2000;
		} else {
			if(i < 6)
				actuators[i] = 1200;
			else
				actuators[i] = 1000;
		}
	}
	if(getElapsedTime() - trigger_time > 0.5) is_triggering = 0;


	if(print_timing) {
	printf("LLA %04.1f  UTC %04.01f  VEL %04.01f  HS %04.01f | MAG %05.01f | STAT %05.01f\n",
			(float)gnss_lla_cnt/getElapsedTime(),
			(float)gnss_utc_cnt/getElapsedTime(),
			(float)gnss_vel_cnt/getElapsedTime(),
			(float)gnss_hs_cnt/getElapsedTime(),
			(float)mag_cnt/getElapsedTime(),
			(float)stat_p_cnt/getElapsedTime()
			);
	}

	if(display_telemetry) {
		for(uint8_t i=0; i<16; i++)
			printf("%04u ",actuators[i]);
		printf(" [%u] \n", is_triggering);
	}

	BRIDGE_SendActuatorPkt(1,actuators);

}

void exitTest() {
	tcsetattr(0, TCSANOW, &initial_settings);
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
