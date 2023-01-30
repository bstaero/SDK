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
#include "file_interface.h"

/* NetUAS */
#include "netuas_serial.h"
#include "netuas_socket.h"

/* STD LIBS */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>

#ifdef __APPLE__
#include <mach/mach_time.h> // system time
#endif

#ifdef VERBOSE
#  include "debug.h"
#endif

/*<---Global Variables---->*/
CommunicationsProtocol * comm_handler;
CommunicationsInterface * comm_interface;

int out_fid = -1;

SystemStatus_t system_status;
SystemInitialize_t system_initialize;
/*<-End Global Variables-->*/

bool big_endian = false;
bool running = true;

void printHelp();

uint8_t comm_type = COMM_UNKNOWN;

int main(int argc, char *argv[])
{
#ifdef VERBOSE
	verbose = VERBOSE_ALL;
#endif

	uint16_t temp = 0x0100;
	big_endian = ((uint8_t *)&temp)[0];

	char param[3][32];

	char outfile[132];

	bzero(outfile,132);

	char c;
	while ((c = getopt(argc, argv, "b:d:i:f:o:p:st:x:h")) != -1) {
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
			case 'f':
				comm_type = COMM_FILE;
				strcpy(&param[0][0],optarg);
				break;
			case 'o':
				strcpy(outfile,optarg);
				break;
			case 's':
				simulation = true;
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
			if(!param[1][0]) strcpy(&param[1][0],"921600");
			break;
		case COMM_SOCKET:
			comm_type = COMM_SOCKET;
			if(!param[0][0]) strcpy(&param[0][0],"localhost");
			if(!param[1][0]) strcpy(&param[1][0],"55555");
			if(!param[2][0]) strcpy(&param[2][0],"TCP:CLIENT");
			break;
		case COMM_FILE:
			comm_type = COMM_FILE;
			if(!param[0][0]) {printHelp(); exit(1);}
			break;
		case COMM_UNKNOWN:
		default:
			comm_type = COMM_SOCKET;
			if(!param[0][0]) strcpy(&param[0][0],"localhost");
			if(!param[1][0]) strcpy(&param[1][0],"55555");
			if(!param[2][0]) strcpy(&param[2][0],"TCP:CLIENT");
			break;
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
	} else if(comm_type == COMM_FILE) {
		comm_handler->setInterface(new FileInterface);
		comm_handler->setAddressing(false);
	}

	comm_interface = comm_handler->getInterface();
	if(comm_interface != NULL)
		comm_interface->initialize(param[0],param[1],param[2]);

	BSTModuleBasic basic_module;
	BSTModuleFlightPlan flight_plan_module((char *)"SDK");

	basic_module.registerReceive(receive);
	basic_module.registerReceiveCommand(receiveCommand);
	basic_module.registerReceiveReply(receiveReply);
	basic_module.registerPublish(publish);

	//flight_plan_module.registerReceive(receive);
	//flight_plan_module.registerReceiveCommand(receiveCommand);
	//flight_plan_module.registerReceiveReply(receiveReply);
	//flight_plan_module.registerPublish(publish);

	((BSTProtocol *)comm_handler)->registerModule(&basic_module);
	//((BSTProtocol *)comm_handler)->registerModule(&flight_plan_module);

	comm_handler->getInterface()->open();

	if(strlen(outfile)) {
		out_fid = open(outfile, O_WRONLY | O_CREAT | O_TRUNC, 0666);
		if(out_fid < 0) {
			printf("ERROR - unable to open file %s for writing.\n",outfile);
			close(out_fid);
			exit(1);
		}
		write_file = true;
	}

	initializeTest();
	printTestHelp();

	float last_connection = getElapsedTime();

	while(running) {
		// Update communications
		if(comm_interface->isConnected()) {
			comm_handler->update();
		} else {
			if(comm_type == COMM_FILE) break;
			if(getElapsedTime() - last_connection > 1.0) {
				comm_handler->getInterface()->open();
				last_connection = getElapsedTime();
			}
		}

		// Perform user functions
		updateTest();

		if(comm_type != COMM_FILE)
			usleep(1000);
	}

	comm_handler->getInterface()->close();

	if(out_fid >= 0) {
		close(out_fid);
	}

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
	printf("  File paramerters:\n");
	printf("    -f <filename>           : read data from file\n");
	printf("    -o <filename>           : save data to file\n");
	printf("  Simulation paramerters:\n");
	printf("    -s                      : simluate data\n");
	exit(0);
}


bool writeFile(uint8_t * data, uint16_t num) {
	return write(out_fid, data, num);
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
