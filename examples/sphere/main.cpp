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

/* BST */
#include "bst_serial.h"
#include "bst_socket.h"

/* STD LIBS */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#ifdef VERBOSE
#  include "debug.h"
#endif

/*<---Global Variables---->*/
CommunicationsProtocol * comm_handler;
CommunicationsInterface * comm_interface;

SystemStatus_t system_status;
SystemInitialize_t system_initialize;
/*<-End Global Variables-->*/

int main(int argc, char *argv[])
{
#ifdef VERBOSE
	verbose = VERBOSE_ALL;
#endif

	detectEndianness();

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
				printBaseHelp();
				exit(0);
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
		printBaseHelp();
		exit(1);
	}

	setupTime();

	// get handler
	comm_handler = new BSTProtocol();

	// set interface
	if(comm_type == COMM_SERIAL) {
		comm_handler->setInterface(new BSTSerial);
	} else if(comm_type == COMM_SOCKET) {
		comm_handler->setInterface(new BSTSocket);
	}

	comm_interface = comm_handler->getInterface();
	if(comm_interface != NULL)
		comm_interface->initialize(param[0],param[1],param[2]);

	BSTModuleBasic basic_module;
	BSTModuleFlightPlan flight_plan_module("SDK");

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

	initTerminal();
	printTestHelp();

	while(comm_interface->isConnected() && running) {
		// Update communications
		comm_handler->update();

		// Perform user functions
		updateTest();

		usleep(1000);
	}

	comm_handler->getInterface()->close();

	restoreTerminal();
	printf("Disconnected, exiting.\n\n");
}
