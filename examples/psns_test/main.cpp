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

/* BST */
#include "bst_serial.h"
#include "bst_socket.h"

/* STD LIBS */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

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

uint8_t comm_type = COMM_UNKNOWN;

void printHelp();

int main(int argc, char *argv[])
{
#ifdef VERBOSE
	//verbose = -VERBOSE_CAN;
	verbose = VERBOSE_ERROR;
#endif

	detectEndianness();

	char param[3][32];

	char outfile[132];

	bzero(outfile,132);

	char c;
	while ((c = getopt(argc, argv, "b:d:i:o:p:t:x:h")) != -1) {
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
			case 'o':
				strcpy(outfile,optarg);
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
		strcpy(&param[1][0],"55556");
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
		comm_handler->setInterface(new BSTSerial);
	} else if(comm_type == COMM_SOCKET) {
		comm_handler->setInterface(new BSTSocket);
	}

	comm_interface = comm_handler->getInterface();
	if(comm_interface != NULL)
		comm_interface->initialize(param[0],param[1],param[2]);

	if(strlen(outfile)) {
		out_fid = open(outfile, O_WRONLY | O_CREAT | O_TRUNC, 0666);
		if(out_fid < 0) {
			printf("ERROR - unable to open file %s for writing.\n",outfile);
			close(out_fid);
			exit(1);
		}
		write_file = true;
	}

	setupSimulatedCAN(comm_interface);

	initTerminal();
	printTestHelp();

	while(running) {
		// Update communications
		simulatedCANRead(1);

		// Perform user functions
		updateTest();

		usleep(1000);
	}

	comm_handler->getInterface()->close();

	if(out_fid >= 0) {
		close(out_fid);
	}

	restoreTerminal();
	printf("Disconnected, exiting.\n\n");
}

void printHelp() {
	printBaseHelp();
	printf("  File parameters:\n");
	printf("    -f <input file> \n");
	printf("\n");
	printf("  -h        Print this help\n");
	exit(0);
}

bool writeBytes(uint8_t * data, uint16_t num) {
	if(comm_type == COMM_SERIAL || comm_type == COMM_SOCKET)
		return comm_interface->write(data, num) == num;

	return false;
}

bool writeFile(uint8_t * data, uint16_t num) {
	return write(out_fid, data, num);
}

