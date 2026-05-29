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
//
// payload_monitor - a terminal "bus monitor" for the BST protocol.
//
// It connects to an autopilot (or SWIL sim) over serial or socket exactly like
// the other SDK examples, but instead of acting as a payload it simply listens
// to every packet that arrives and presents a live, UAVCAN-GUI-Tool-style view:
// a table of every packet type seen with its receive rate (Hz), and a detail
// pane that decodes / hex-dumps the most recent packet of the selected type.
//
// All of the monitoring/TUI logic lives in monitor.{h,cpp}; main.cpp and the
// test*.cpp files are just the standard SDK example plumbing (transport setup
// and the receive hook). See the sibling "payload" example for the payload
// skeleton this is based on.
//
#include "main.h"
#include "test.h"
#include "test_handler.h"
#include "monitor.h"

#include "bst_module_basic.h"
#include "bst_module_flight_plan.h"
#include "bst_protocol.h"

#include "bst_serial.h"
#include "bst_socket.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#ifdef VERBOSE
#  include "debug.h"
#endif

CommunicationsProtocol * comm_handler;
CommunicationsInterface * comm_interface;

SystemStatus_t system_status;
SystemInitialize_t system_initialize;

void printHelp();

int main(int argc, char *argv[])
{
#ifdef VERBOSE
	// Silence all library pmesg() output: it writes to stdout and would scroll /
	// corrupt the full-screen monitor TUI.
	verbose = VERBOSE_NONE;
#endif

	detectEndianness();

	uint8_t comm_type = COMM_UNKNOWN;
	char param[3][32];

	int c;
	while ((c = getopt(argc, argv, "b:d:i:p:h")) != -1) {
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

	// handle bare IP address as positional argument
	if(optind < argc) {
		strcpy(&param[0][0],argv[optind]);
		comm_type != COMM_SERIAL ? comm_type = COMM_SOCKET : comm_type = COMM_INVALID;
		strcpy(&param[2][0],"TCP:CLIENT");
	}

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

	comm_handler = new BSTProtocol();

	char conn[80];
	if(comm_type == COMM_SERIAL) {
		comm_handler->setInterface(new BSTSerial);
		snprintf(conn, sizeof(conn), "serial %s @ %s", param[0], param[1]);
	} else if(comm_type == COMM_SOCKET) {
		comm_handler->setInterface(new BSTSocket);
		snprintf(conn, sizeof(conn), "tcp %s:%s", param[0], param[1]);
	}

	comm_interface = comm_handler->getInterface();
	if(comm_interface != NULL)
		comm_interface->initialize(param[0],param[1],param[2]);

	// Receive promiscuously.  BSTProtocol::update() drops any addressed packet
	// whose destination doesn't match our serial_num (via Packet::isToID), and a
	// serial_num of 0 (== NO_ID) matches *nothing* -- not even ALL_NODES
	// broadcasts.  Setting our address to ALL_NODES makes isToID() match every
	// packet, so the monitor sees all traffic on the link regardless of who it
	// is addressed to.
	system_initialize.serial_num = ALL_NODES;

	BSTModuleBasic basic_module;
	BSTModuleFlightPlan flight_plan_module((char *)"SDK");

	basic_module.registerReceive(receive);
	basic_module.registerReceiveCommand(receiveCommand);
	basic_module.registerReceiveReply(receiveReply);
	basic_module.registerPublish(publish);

	((BSTProtocol *)comm_handler)->registerModule(&basic_module);

	comm_handler->getInterface()->open();

	initTerminal();
	monitorInit(conn);

	while(running) {
		comm_handler->update();
		updateTest();
		usleep(1000);
	}

	monitorShutdown();
	comm_handler->getInterface()->close();

	restoreTerminal();
	printf("Disconnected, exiting.\n\n");
}

void printHelp() {
	printBaseHelp();
	exit(0);
}
