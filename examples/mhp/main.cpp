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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#ifdef __APPLE__
#include <mach/mach_time.h> // system time
#endif

/*<---Global Variables---->*/
CommunicationsInterface * comm_interface;
int in_fid = -1;
int out_fid = -1;
/*<-End Global Variables-->*/

enum {COMM_SERIAL, COMM_SOCKET, COMM_FILE, COMM_UNKNOWN, COMM_INVALID};
uint8_t comm_type = COMM_UNKNOWN;

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

	char param[3][32];
	param[0][0] = 0;
	param[1][0] = 0;
	param[2][0] = 0;

	char infile[132];
	char outfile[132];

	bzero(infile,132);
	bzero(outfile,132);

	int8_t c;
	int8_t retval = -1;
	while ((c = getopt(argc, argv, "b:d:f:i:o:p:t:x:h")) != retval) {
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
				strcpy(infile,optarg);
				break;
			case 'o':
				strcpy(outfile,optarg);
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
			if(!strlen(infile)) {printHelp(); exit(1);}
			break;
		case COMM_UNKNOWN:
		default:
			comm_type = COMM_SERIAL;
			if(!param[0][0]) strcpy(&param[0][0],"/dev/ttyUSB0");
			if(!param[1][0]) strcpy(&param[1][0],"921600");
			break;
	}

	if(comm_type == COMM_INVALID) {
		printHelp();
		exit(1);
	}

	setupTime();

	// set interface
	if(comm_type == COMM_SERIAL || comm_type == COMM_SOCKET) {
		if(comm_type == COMM_SERIAL) {
			comm_interface = new NetuasSerial;
		} else if(comm_type == COMM_SOCKET) {
			comm_interface = new NetuasSocket;
		}

		comm_interface->initialize(param[0],param[1],param[2]);
		comm_interface->open();
	}

	if(comm_type == COMM_FILE) {
		in_fid = open(infile, O_RDONLY);
		if(in_fid < 0) {
			printf("ERROR - unable to open file %s for reading.\n",infile);
			exit(1);
		}
	}

	if(strlen(outfile)) {
		out_fid = open(outfile, O_WRONLY | O_CREAT | O_TRUNC, 0666);
		if(out_fid < 0) {
			printf("ERROR - unable to open file %s for writing.\n",outfile);
			close(in_fid);
			exit(1);
		}
		write_file = true;
	}

	initializeTest();

	if(comm_type == COMM_SERIAL || comm_type == COMM_SOCKET) {
		while(comm_interface->isConnected() && running) {
			// Update communications
			updateCommunications();

			// Perform user functions
			updateTest();

			usleep(100);
		}
	}

	if(comm_type == COMM_FILE) {
		while(updateCommunications() && running) {
			// Perform user functions
			updateTest();

			if(out_fid < 0)
				usleep(100);
		}
	}

	if(comm_type == COMM_SERIAL || comm_type == COMM_SOCKET)
		comm_interface->close();

	if(comm_type == COMM_FILE) {
		close(in_fid);
		close(out_fid);
	}

	exitTest();
	printf("Disconnected, exiting.\n\n");
}

void printHelp() {
	printf("Usage: test [OPTIONS]\n");
	printf("  Serial port paramerters:\n");
	printf("    -d <serial device name> : default /dev/ttyUSB0\n");
	printf("    -b <serial baud>        : default 921600\n");
	printf("  Socket paramerters:\n");
	printf("    -i <server ip number>   : default localhost\n");
	printf("    -p <socket port number> : default 55555\n");
	printf("  File paramerters:\n");
	printf("    -f <input file> \n");
	printf("    -o <output file> \n");
	printf("\n");
	printf("  -h        Print this help\n");
	exit(0);
}

bool readByte(uint8_t * data) {
	if(comm_type == COMM_SERIAL || comm_type == COMM_SOCKET)
		return comm_interface->read(data,1) > 0;

	if(comm_type == COMM_FILE)
		return read(in_fid, data, 1);

	return false;
}

bool writeBytes(uint8_t * data, uint16_t num) {
	if(comm_type == COMM_SERIAL || comm_type == COMM_SOCKET)
		return comm_interface->write(data, num) == num;

	return false;
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
