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
bool show_gps = false;
bool show_mag = false;
bool show_dynamic = false;
bool show_static = false;

bool new_gps = false;
bool new_mag = false;
bool new_dynamic = false;
bool new_static = false;

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

extern GPS_t                  gps;
extern ThreeAxisSensor_t      mag;
extern Pressure_t             dyn_p;
extern Pressure_t             stat_p;

// for command line (terminal) input
struct termios initial_settings, new_settings;

void printTestHelp() {
	printf("Keys:\n");
	printf("  g   : Toggle GPS Display\n");
	printf("  m   : Toggle Magnetometer Display\n");
	printf("  d   : Toggle Dynamic Pressure Display\n");
	printf("  s   : Toggle Static Pressure Display\n");
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
				case 'g':
					show_gps = !show_gps;
					break;

				case 'm':
					show_mag = !show_mag;
					break;

				case 'd':
					show_dynamic = !show_dynamic;
					break;

				case 's':
					show_static = !show_static;
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

	if(show_gps) {
		if(new_gps) {
			printf("GPS: %u %02u:%02u:%04.1f %+06.02f,%+07.02f,%05.01f %+05.01f %05.01f %02u %05.01f %03.01f %u %+05.02f,%+05.02f,%+05.02f\n",
					gps.week,
					gps.hour,
					gps.minute,
					gps.seconds,
					gps.latitude,
					gps.longitude,
					gps.altitude,
					gps.speed,
					gps.course,
					gps.satellites,
					gps.pdop,
					gps.last_fix,
					gps.fix_type,
					gps.velocity.x,
					gps.velocity.y,
					gps.velocity.z);
			new_gps = false;
		}
	}

	if(show_mag) {
		if(new_mag) {
			printf("MAG: %+05.02f %+05.02f %+05.02f\n",
					mag.x,
					mag.y,
					mag.z);
			new_mag = false;
		}
	}

	if(show_dynamic) {
		if(new_dynamic) {
			printf("DYNAMIC: %07.01f HPa %05.02f Deg C\n",
					dyn_p.pressure,
					dyn_p.temperature);
			new_dynamic = false;
		}
	}

	if(show_static) {
		if(new_static) {
			printf("STATIC: %07.01f HPa %05.02f Deg C\n",
					stat_p.pressure,
					stat_p.temperature);
			new_static = false;
		}
	}

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

		comm_handler->send(PAYLOAD_CHANNEL_0, (uint8_t *)&tx_payload, sizeof(UserPayload_t), NULL);

		ptr += tx_payload.size;
		tx_payload.size = 0;
		for(uint8_t i=0; i<64; i++) tx_payload.buffer[i] = 0;
	}

	return true;
}
