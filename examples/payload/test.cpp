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

#include "test.h"
#include "main.h"
#include "structs.h"

#include "flight_plan.h"

// variables
bool show_telemetry = false;

// functional definitions
bool sendPayloadData(uint8_t * data, uint8_t size);

// packet for transmision
Packet              tx_packet;

extern TelemetryOrientation_t telemetry_orientation;
extern TelemetryPosition_t    telemetry_position;
extern TelemetryPressure_t    telemetry_pressure;
extern TelemetrySystem_t      telemetry_system;
extern TelemetryControl_t     telemetry_control;

extern UserPayload_t          rx_payload;
UserPayload_t                 tx_payload;

// for command line (terminal) input
struct termios initial_settings, new_settings;

void printTestHelp() {
	printf("Keys:\n");
	printf("  c   : Send payload control data\n");
	printf("  t   : Toggle Telemetry Display\n");
	printf("\n");
	printf("  p   : print this help\n");
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
	char input; 

	Command_t command;

	uint8_t num_points = 0;
	Waypoint_t temp_waypoint;
	Waypoint_t temp_waypoints[MAX_WAYPOINTS];

	FlightPlanMap_t flight_plan_map;
	FlightPlan flight_plan;


	if( inputAvailable() ) {
		input = getchar();

		if(input > 0) {
			switch(input) {
				case 'c':
					printf("Sending payload data\n");

					uint8_t data[128];
					sprintf((char*)data,"This is a test %07.01f\n\r",getElapsedTime());
					sendPayloadData(data,strlen((char*)data));
					break;

				case 't':
					show_telemetry = !show_telemetry;
					break;

				case 'p':
					printTestHelp();
					break;

				case 3: // <CTRL-C> 
					// allow flowthrough
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

	// show telemetry

	if(show_telemetry)
		printf("lla: %+06.02f %+07.02f %06.01f | %08.01f Pa %04.01f deg %04.01f %% <%+05.01f,%+05.01f,%+05.01f>\n",
				telemetry_position.latitude,
				telemetry_position.longitude,
				telemetry_position.altitude,
				telemetry_pressure.static_pressure,
				telemetry_pressure.air_temperature,
				telemetry_pressure.humidity,
				telemetry_pressure.wind.x,
				telemetry_pressure.wind.y,
				telemetry_pressure.wind.z
				);
}

void exitTest() {
	tcsetattr(0, TCSANOW, &initial_settings);
}

bool sendPayloadData(uint8_t * data, uint8_t size) {
	uint8_t ptr = 0;

	while(ptr < size) {
		if(size-ptr > 64) tx_payload.size = 64;
		else tx_payload.size = size-ptr;

		memcpy(tx_payload.buffer,&(data[ptr]),tx_payload.size);

		comm_handler->send(PAYLOAD_DATA_CHANNEL_0, (uint8_t *)&tx_payload, sizeof(UserPayload_t), NULL);

		ptr += tx_payload.size;
		tx_payload.size = 0;
		for(uint8_t i=0; i<64; i++) tx_payload.buffer[i] = 0;
	}

	return true;
}
