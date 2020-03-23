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
#include "main.h"
#include "test.h"
#include "test_handler.h"

/* BST */
#include "bst_packet.h"

/* NetUAS */
#include "netuas_serial.h"
#include "netuas_socket.h"

/* STD LIBS */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

#ifdef __APPLE__
#include <mach/mach_time.h> // system time
#endif

#ifdef VERBOSE
#  include "debug.h"
#endif

/*<---Global Variables---->*/
CommunicationsInterface * comm_interface;
/*<-End Global Variables-->*/

enum {COMM_SERIAL, COMM_SOCKET, COMM_UNKNOWN, COMM_INVALID};

bool big_endian = false;
bool running = true;

void printHelp();

int main(int argc, char *argv[])
{
#ifdef VERBOSE
	verbose = VERBOSE_ALL;
#endif

	uint16_t temp = 0x0100;
	big_endian = ((uint8_t *)&temp)[0];

	uint8_t comm_type = COMM_UNKNOWN;

	char param[3][32];
	param[0][0] = 0;
	param[1][0] = 0;
	param[2][0] = 0;

	char c;
	int retval = -1;
	while ((c = getopt(argc, argv, "b:d:i:p:t:x:h")) != retval) {
		switch(c) {
			case 'b':
				strcpy(&param[1][0],optarg);
				comm_type != COMM_SOCKET ? comm_type = COMM_SERIAL : comm_type = COMM_INVALID;
				break;
			case 'd':
				strcpy(&param[0][0],optarg);
				comm_type != COMM_SOCKET ? comm_type = COMM_SERIAL : comm_type = COMM_INVALID;
				break;
			case 'i':
				strcpy(&param[0][0],optarg);
				comm_type != COMM_SERIAL ? comm_type = COMM_SOCKET : comm_type = COMM_INVALID;
				strcpy(&param[2][0],"TCP:CLIENT");
				break;
			case 'p':
				strcpy(&param[1][0],optarg);
				comm_type != COMM_SERIAL ? comm_type = COMM_SOCKET : comm_type = COMM_INVALID;
				strcpy(&param[2][0],"TCP:CLIENT");
				break;
			default:
				printHelp();
				break;
		}
	}

	switch(comm_type) {
		case COMM_SERIAL:
			comm_type = COMM_SERIAL;
			if(!param[0][0]) strcpy(&param[0][0],"/dev/ttyUSB0");
			if(!param[1][0]) strcpy(&param[1][0],"115200");
			break;
		case COMM_SOCKET:
			comm_type = COMM_SOCKET;
			if(!param[0][0]) strcpy(&param[0][0],"localhost");
			if(!param[1][0]) strcpy(&param[1][0],"55555");
			if(!param[2][0]) strcpy(&param[2][0],"TCP:CLIENT");
			break;
		case COMM_UNKNOWN:
		default:
			comm_type = COMM_SERIAL;
			if(!param[0][0]) strcpy(&param[0][0],"/dev/ttyUSB0");
			if(!param[1][0]) strcpy(&param[1][0],"115200");
			break;
	}

	if(comm_type == COMM_INVALID) {
		printHelp();
		exit(1);
	}

	// set interface
	if(comm_type == COMM_SERIAL) {
		comm_interface = new NetuasSerial;
	} else if(comm_type == COMM_SOCKET) {
		comm_interface = new NetuasSocket;
	}

	comm_interface->initialize(param[0],param[1],param[2]);
	comm_interface->open();

	initializeTest();

	while(comm_interface->isConnected() && running) {
		// Update communications
		updateCommunications();

		// Perform user functions
		updateTest();

		usleep(100);
	}

	comm_interface->close();

	exitTest();
	printf("Disconnected, exiting.\n\n");
}

void printHelp() {
	printf("Usage: test [OPTIONS]\n");
	printf("  Serial port paramerters:\n");
	printf("    -d <serial device name> : default /dev/ttyUSB0\n");
	printf("    -b <serial baud>        : default 115200\n");
	printf("  Socket paramerters:\n");
	printf("    -i <server ip number>   : default localhost\n");
	printf("    -p <socket port number> : default 55555\n");
	printf("\n");
	printf("  -h        Print this help\n");
	exit(0);
}

bool readByte(uint8_t * data) {
	return comm_interface->read(data,1) > 0;
}

bool writeBytes(uint8_t * data, uint16_t num) {
	return comm_interface->write(data, num) == num;
}
