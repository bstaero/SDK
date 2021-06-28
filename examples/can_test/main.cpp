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
#include "bst_module_basic.h"
#include "bst_module_flight_plan.h"
#include "bst_protocol.h"
#include "helper_functions.h"

#include "simulated_can.h"

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
CommunicationsProtocol * comm_handler;
CommunicationsInterface * comm_interface;

SystemStatus_t system_status;
SystemInitialize_t system_initialize;
/*<-End Global Variables-->*/

enum {COMM_SERIAL, COMM_SOCKET, COMM_UNKNOWN, COMM_INVALID};

bool big_endian = false;
bool running = true;

void printHelp();

int main(int argc, char *argv[])
{
#ifdef VERBOSE
	//verbose = VERBOSE_CAN;
	verbose = VERBOSE_ERROR;
#endif

	uint16_t temp = 0x0100;
	big_endian = ((uint8_t *)&temp)[0];

	uint8_t comm_type = COMM_UNKNOWN;

	char param[3][32];

	char c;
	while ((c = getopt(argc, argv, "b:d:i:p:t:x:h")) != -1) {
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

	// set default
	if(comm_type == COMM_UNKNOWN) {
		comm_type = COMM_SOCKET;
		strcpy(&param[0][0],"localhost");
		strcpy(&param[1][0],"55555");
		strcpy(&param[2][0],"TCP:CLIENT");
	}

	if(comm_type == COMM_INVALID) {
		printHelp();
		exit(1);
	}

	setupTime();

	// get handler
	comm_handler = new BSTProtocol();

	// set interface
	if(comm_type == COMM_SERIAL) {
		comm_handler->setInterface(new NetuasSerial);
	} else if(comm_type == COMM_SOCKET) {
		comm_handler->setInterface(new NetuasSocket);
	}

	comm_interface = comm_handler->getInterface();
	if(comm_interface != NULL)
		comm_interface->initialize(param[0],param[1],param[2]);

	setupSimulatedCAN(comm_interface);

	initializeTest();
	printTestHelp();

	while(comm_interface->isConnected() && running) {
		// Update communications
		simulatedCANRead();

		// Perform user functions
		updateTest();

		usleep(1000);
	}

	comm_handler->getInterface()->close();

	exitTest();
	printf("Disconnected, exiting.\n\n");
}

void printHelp() {
	printf("Usage: test [OPTIONS]\n");
	printf("  Serial port paramerters:\n");
	printf("    -d <serial device name> : default /dev/ttyUSB0\n");
	printf("    -b <serial baud>        : default 9600\n");
	printf("  Socket paramerters:\n");
	printf("    -i <server ip number>   : default localhost\n");
	printf("    -p <socket port number> : default 55552\n");
	exit(0);
}

double start_time = 0.0;

void setupTime() {
#ifdef __APPLE__
	uint64_t now = mach_absolute_time();
	float conversion  = 0.0;
	mach_timebase_info_data_t info;
	kern_return_t err = mach_timebase_info( &info );
	if( err == 0  )
		conversion = 1e-9 * (float) info.numer / (float) info.denom;
	start_time = conversion * (float) now;
#else
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	start_time = (double)now.tv_sec + (double)now.tv_nsec / SEC_TO_NSEC;
#endif
}

float getElapsedTime() {
#ifdef __APPLE__
	uint64_t now = mach_absolute_time();
	float conversion  = 0.0;
	mach_timebase_info_data_t info;
	kern_return_t err = mach_timebase_info( &info );
	if( err == 0  )
		conversion = 1e-9 * (float) info.numer / (float) info.denom;
	float current_time = conversion * (float) now;
#else
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	double current_time = (double)now.tv_sec + (double)now.tv_nsec / SEC_TO_NSEC;
#endif
	return current_time - start_time;
}
