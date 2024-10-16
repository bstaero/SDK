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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>

#define VEHICLE_MULTIROTOR

#include "test.h"
#include "test_handler.h"

#include "main.h"

/*<---Global Variables---->*/
volatile bool display_telemetry = false;
volatile bool display_telemetry_timing = false;
volatile bool write_file = false;

// for command line (terminal) input
struct termios initial_settings, new_settings;
/*<-End Global Variables-->*/

/*<---Local Functions----->*/
void updateCalibration(void);
void updateOrientation(void);
/*<-End Local Functions--->*/

void printTestHelp() {
	printf("Keys:\n");
	printf("  t   : Toggle telemetry display\n");
	printf("  i   : Toggle telemetry timing display\n");
	printf("\n");
	printf("  s   : Request serial number and comms revision\n");
	printf("\n");
	printf("  p   : Print this help\n");
}

bool inputAvailable()  
{
	// check for input on terminal
	struct timeval tv;
	fd_set fds;
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds);
	select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);

	return (FD_ISSET(0, &fds));
}


void initializeTest() {

	// terminal settings to get input
	tcgetattr(0,&initial_settings);

	new_settings = initial_settings;
	new_settings.c_lflag &= ~ICANON;
	new_settings.c_lflag &= ~ECHO;
	new_settings.c_lflag &= ~ISIG;
	new_settings.c_cc[VMIN] = 0;
	new_settings.c_cc[VTIME] = 0;

	tcsetattr(0, TCSANOW, &new_settings);
}

void updateTest() {
	static bool first_run = true;
	if(first_run) {
		printTestHelp();
		first_run = false;
	}

	AxisMapping_t temp_axis_mapping;
	int temp_int = 0;

	char input; 

	if( inputAvailable() ) {
		input = getchar();

		if(input > 0) {
			switch(input) {

				case 't':
					display_telemetry_timing = false;
					display_telemetry? display_telemetry=false: display_telemetry=true;
					break;

				case 'i':
					display_telemetry = false;
					display_telemetry_timing? display_telemetry_timing=false: display_telemetry_timing=true;
					break;

				case 's':
					requestPowerOn();
					break;

				case 'p':
					printTestHelp();
					break;

				case 3: // <CTRL-C> 
				case 'q':
					printf("Keyboard caught exit signal ...\n");
					running = false;
					break;

				default:
					break;
			}
			input = 0;
		} else {
			clearerr(stdin);
		}
	}

}

void exitTest() {
	tcsetattr(0, TCSANOW, &initial_settings);
}
