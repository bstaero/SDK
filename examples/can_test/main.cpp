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
#include "log_replay.h"

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
#include <getopt.h>

#ifdef VERBOSE
#  include "debug.h"
#endif

/*<---Global Variables---->*/
CommunicationsProtocol * comm_handler;
CommunicationsInterface * comm_interface;

int out_fid = -1;

SystemStatus_t system_status;
SystemInitialize_t system_initialize;

extern "C"  {
	uint8_t p_new_gps_data = 0;
}

extern bool auto_test;

char log_filename[256] = {0};
/*<-End Global Variables-->*/

// parse "hh:mm:ss" or "mm:ss" or bare seconds into float seconds
static float parseTimeStr(const char * str) {
	int h = 0, m = 0;
	float s = 0.0f;
	if(sscanf(str, "%d:%d:%f", &h, &m, &s) == 3) {
		return h * 3600.0f + m * 60.0f + s;
	}
	h = 0;
	if(sscanf(str, "%d:%f", &m, &s) == 2) {
		return m * 60.0f + s;
	}
	if(sscanf(str, "%f", &s) == 1) {
		return s;
	}
	return -1.0f;
}

int main(int argc, char *argv[])
{
#ifdef VERBOSE
	verbose = -VERBOSE_CAN;
	//verbose = VERBOSE_ERROR;
#endif

	detectEndianness();

	uint8_t comm_type = COMM_UNKNOWN;

	char param[3][32];

	char outfile[132];

	bzero(outfile,132);

	float dd_duration = -1.0f;

	static struct option long_options[] = {
		{"ss", required_argument, 0, 0x100},
		{"tt", required_argument, 0, 0x101},
		{"dd", required_argument, 0, 0x102},
		{0, 0, 0, 0}
	};

	int c;
	int option_index = 0;
	while ((c = getopt_long(argc, argv, "ab:d:f:i:o:p:t:x:h", long_options, &option_index)) != -1) {
		switch(c) {
			case 'a':
				auto_test = true;
				break;
			case 'b':
				strcpy(&param[1][0],optarg);
				comm_type != COMM_SOCKET ? comm_type = COMM_SERIAL : comm_type = COMM_INVALID;
				break;
			case 'd':
				strcpy(&param[0][0],optarg);
				comm_type != COMM_SOCKET ? comm_type = COMM_SERIAL : comm_type = COMM_INVALID;
				break;
			case 'f':
				strncpy(log_filename, optarg, sizeof(log_filename)-1);
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
			case 0x100: // --ss
				replay_start_s = parseTimeStr(optarg);
				break;
			case 0x101: // --tt
				replay_stop_s = parseTimeStr(optarg);
				break;
			case 0x102: // --dd
				dd_duration = parseTimeStr(optarg);
				break;
			default:
				printHelp();
				break;
		}
	}

	// --dd converts to --tt (stop = start + duration)
	if(dd_duration >= 0.0f && replay_stop_s < 0.0f) {
		float ss = (replay_start_s >= 0.0f) ? replay_start_s : 0.0f;
		replay_stop_s = ss + dd_duration;
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
		// Perform user functions first for responsive keyboard handling
		updateTest();

		// Update communications
		simulatedCANRead(1);

		usleep(1000);
	}

	// connection lost or user quit – zero actuators before closing
	zeroAcutators();

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
	printf("    -f <log file>           : BST binary log for actuator replay\n");
	printf("    --ss hh:mm:ss           : replay start time (relative to log start)\n");
	printf("    --tt hh:mm:ss           : replay stop time (relative to log start)\n");
	printf("    --dd hh:mm:ss           : replay duration (from start time)\n");
	printf("\n");
	printf("  -h        Print this help\n");
	exit(0);
}

bool writeFile(uint8_t * data, uint16_t num) {
	return write(out_fid, data, num);
}
