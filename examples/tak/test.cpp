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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include "test.h"
#include "main.h"

#define DISPLAY_PERIOD 1.0  // [s]

bool show_telemetry = true;

TakClient    tak;
float        tak_period = 1.0;
float        tak_stale  = 30.0;
const char * tak_type   = "a-f-A-M-F-Q";  // friendly / air / military / fixed wing / UAV

static float last_display = 0.0;

void printTestHelp() {
	printf("Keys:\n");
	printf("  t   : Toggle aircraft display\n");
	printf("  p   : print this help\n");
	printf("  q   : quit\n");
	printf("\n");
}

static void sendToTak(uint32_t address, Aircraft_t & ac) {
	const TelemetryPosition_t & pos = ac.position;

	double lat = pos.latitude  / 1e16;
	double lon = pos.longitude / 1e16;
	if(lat == 0.0 && lon == 0.0) return;  // no GPS fix yet

	// velocity[0..1] are north/east [m/s * 100]
	double vn = pos.velocity[0] / 100.0;
	double ve = pos.velocity[1] / 100.0;
	double course = atan2(ve, vn) * 180.0 / M_PI;
	if(course < 0.0) course += 360.0;

	char uid[32], callsign[32], remarks[128];
	snprintf(uid, sizeof(uid), "BST-%08X", address);
	if(ac.name[0] != '\0')
		snprintf(callsign, sizeof(callsign), "%s", ac.name);
	else
		snprintf(callsign, sizeof(callsign), "BST-%08X", address);
	snprintf(remarks, sizeof(remarks), "AGL %.1f m, batt %.0f%%, sats %u",
			pos.height / 1000.0, ac.system.batt_percent / 100.0, ac.system.satellites);

	TakTrack_t track;
	track.uid       = uid;
	track.callsign  = callsign;
	track.type      = tak_type;
	track.latitude  = lat;
	track.longitude = lon;
	track.altitude  = pos.altitude / 1000.0;  // MSL, see README
	track.course    = course;
	track.speed     = sqrt(vn*vn + ve*ve);
	track.stale     = tak_stale;
	track.remarks   = remarks;

	tak.send(track);
}

void updateTest() {
	char input;

	if( inputAvailable() ) {
		input = getchar();

		if(input > 0) {
			switch(input) {
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
		} else {
			clearerr(stdin);
		}
	}

	float now = getElapsedTime();

	// forward each aircraft's newest position, at most once per tak_period
	for(std::map<uint32_t, Aircraft_t>::iterator it = aircraft.begin(); it != aircraft.end(); ++it) {
		Aircraft_t & ac = it->second;
		if(!ac.have_position || ac.last_rx <= ac.last_tak_tx) continue;
		if(now - ac.last_tak_tx < tak_period) continue;

		sendToTak(it->first, ac);
		ac.last_tak_tx = now;
	}

	if(show_telemetry && now - last_display >= DISPLAY_PERIOD) {
		last_display = now;

		if(aircraft.empty())
			printf("waiting for aircraft telemetry ...\n");

		for(std::map<uint32_t, Aircraft_t>::iterator it = aircraft.begin(); it != aircraft.end(); ++it) {
			const Aircraft_t & ac = it->second;
			printf("0x%08X %-16s lla: %+11.7f %+12.7f %7.1f m | age %4.1f s | TAK %s\n",
					it->first, ac.name,
					ac.position.latitude / 1e16,
					ac.position.longitude / 1e16,
					ac.position.altitude / 1000.0,
					now - ac.last_rx,
					tak.isConnected() ? "up" : "down");
		}
	}
}
