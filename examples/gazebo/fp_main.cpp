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
#include "fp_main.h"
#include "fp_test.h"
#include "test_handler.h"

/* BST */
#include "bst_module_basic.h"
#include "bst_module_flight_plan.h"
#include "bst_protocol.h"
#include "helper_functions.h"

/* NetUAS */
#include "netuas_serial.h"
#include "netuas_socket.h"

/* STD LIBS */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <thread>
#include <termios.h>

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

size_t numWPs = 100;
size_t stepTime = 5000;

// for command line (terminal) input
struct termios initial_settings, new_settings;

/*<-End Global Variables-->*/

enum {COMM_SERIAL, COMM_SOCKET, COMM_UNKNOWN, COMM_INVALID};

bool big_endian = false;
bool running = true;

std::thread tsendMultipleWP;

void printHelp();

int main(int argc, char *argv[])
{
size_t steps = 0;
#ifdef VERBOSE
	verbose = VERBOSE_INFO;
#endif

	uint16_t temp = 0x0100;
	big_endian = ((uint8_t *)&temp)[0];

	uint8_t comm_type = COMM_UNKNOWN;

	char param[3][32];
	char c;
	
  	
	while ((c = getopt(argc, argv, "n:t:h")) != -1) {
		switch(c) {
			case 'n':
				numWPs = atol(optarg);				
				break;
			case 't':
				stepTime = atol(optarg);
				break;				
			case 'h':
			default:
				printHelp();
				break;
		}
	}

	// set default
	if(comm_type == COMM_UNKNOWN) {
		comm_type = COMM_SOCKET;
		strcpy(&param[0][0],"localhost");
		strcpy(&param[1][0],"55551");
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

	BSTModuleBasic basic_module;
	BSTModuleFlightPlan flight_plan_module;

	basic_module.registerReceive(receive);
	basic_module.registerReceiveCommand(receiveCommand);
	basic_module.registerReceiveReply(receiveReply);
	basic_module.registerPublish(publish);

	flight_plan_module.registerReceive(receive);
	flight_plan_module.registerReceiveCommand(receiveCommand);
	flight_plan_module.registerReceiveReply(receiveReply);
	flight_plan_module.registerPublish(publish);

	//flight_plan_module.setWaypointTimeout(0.1);

	((BSTProtocol *)comm_handler)->registerModule(&basic_module);
	((BSTProtocol *)comm_handler)->registerModule(&flight_plan_module);

	comm_handler->getInterface()->open();
	
	initializeTest();		
	 
	while(comm_interface->isConnected() && running && steps < numWPs ) 
	{			
		// Update communications
		comm_handler->update(); 
	
		// Perform user functions
		steps = sendMultipleWP();	
		usleep(1000);
	}
	printf("isConnected: %d running: %d steps: %ld numWPs: %ld\n", comm_interface->isConnected(), running , steps , numWPs );

	comm_handler->getInterface()->close();

	reset_term();

	printf("goodbye!\n\n");
}

void printHelp() {
	printf("Usage: test [OPTIONS]\n");
	printf("  Number of points:\n");
	printf("    -n <Number of points to send> : default 100\n");	
	printf("  Time between 2 WPs:\n");
	printf("    -t <time WP to WP(ms)>   : default 5000 (ms)\n");	
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

void initializeTest() {

    // terminal settings to get input
    tcgetattr(0, &initial_settings);

    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 0;
    new_settings.c_cc[VTIME] = 0;

    tcsetattr(0, TCSANOW, &new_settings);

    // initialize system_init packet
    system_init.vehicle_type = PAYLOAD_NODE;
    system_init.serial_num = 0x1234;
}

void reset_term() {
    tcsetattr(0, TCSANOW, &initial_settings);
}

void exitTest() {
	printf("-- STOPPING TEST\n");
	running = false;
}