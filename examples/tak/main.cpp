/*=+--+=#=+--         SwiftCore Flight Management Software        --+=#=+--+=#*\
|               Copyright (C) 2026 Black Swift Technologies LLC.               |
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

#include "bst_module_basic.h"
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
	verbose = VERBOSE_ERROR;
#endif

	detectEndianness();

	uint8_t comm_type = COMM_UNKNOWN;
	char param[3][32];

	char tak_host[64] = "localhost";
	char tak_port[16] = "8087";
	TakTransport_t tak_transport = TAK_TCP;

	int c;
	while ((c = getopt(argc, argv, "b:d:i:p:T:P:UR:S:h")) != -1) {
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
			case 'T':
				snprintf(tak_host,sizeof(tak_host),"%s",optarg);
				break;
			case 'P':
				snprintf(tak_port,sizeof(tak_port),"%s",optarg);
				break;
			case 'U':
				tak_transport = TAK_UDP;
				break;
			case 'R':
				tak_period = atof(optarg);
				break;
			case 'S':
				tak_stale = atof(optarg);
				break;
			default:
				printHelp();
				break;
		}
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

	// BSTProtocol drops packets not addressed to our serial number, and a
	// serial of 0 (NO_ID) matches nothing. As a passive listener, accept all.
	system_initialize.serial_num = ALL_NODES;

	comm_handler = new BSTProtocol();

	if(comm_type == COMM_SERIAL) {
		comm_handler->setInterface(new BSTSerial);
	} else if(comm_type == COMM_SOCKET) {
		comm_handler->setInterface(new BSTSocket);
	}

	comm_interface = comm_handler->getInterface();
	if(comm_interface != NULL)
		comm_interface->initialize(param[0],param[1],param[2]);

	BSTModuleBasic basic_module;

	basic_module.registerReceive(receive);
	basic_module.registerReceiveCommand(receiveCommand);
	basic_module.registerReceiveReply(receiveReply);
	basic_module.registerPublish(publish);

	((BSTProtocol *)comm_handler)->registerModule(&basic_module);

	comm_handler->getInterface()->open();

	// failure here is not fatal, the client keeps retrying in the background
	tak.open(tak_host, tak_port, tak_transport);

	initTerminal();
	printTestHelp();

	while(running) {
		comm_handler->update();
		updateTest();
		usleep(1000);
	}

	comm_handler->getInterface()->close();
	tak.close();

	restoreTerminal();
	printf("Disconnected, exiting.\n\n");
}

void printHelp() {
	printBaseHelp();
	printf("  TAK parameters:\n");
	printf("    -T <tak host>           : default localhost\n");
	printf("    -P <tak port>           : default 8087\n");
	printf("    -U                      : send CoT over UDP (e.g. -T 239.2.3.1 -P 6969)\n");
	printf("    -R <seconds>            : min time between updates per aircraft, default 1.0\n");
	printf("    -S <seconds>            : CoT stale time, default 30\n");
	exit(0);
}
