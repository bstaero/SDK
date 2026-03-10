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

#include "test.h"
#include "main.h"
#include "structs.h"

#include "flight_plan.h"
#include "bridge.h"

// variables
volatile bool display_telemetry = false;
volatile bool write_file = false;

bool print_timing = false;

extern uint32_t gnss_lla_cnt;
extern uint32_t gnss_utc_cnt;
extern uint32_t gnss_vel_cnt;
extern uint32_t gnss_hs_cnt;

extern uint32_t mag_cnt;
extern uint32_t stat_p_cnt;

extern CommunicationsInterface * comm_interface;

Packet              tx_packet;

void printTestHelp() {
	printf("Keys:\n");
	printf("  t   : Toggle telemetry display\n");
	printf("  i   : Toggle timing display\n");
	printf("\n");
	printf("  p   : print this help\n");
}

void updateTest() {
	char input;

	if( inputAvailable() ) {
		input = getchar();

		if(input > 0) {
			switch(input) {

				case 't':
					display_telemetry = !display_telemetry;
					break;

				case 'i':
					print_timing = !print_timing;
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

	if(print_timing) {
		printf("LLA %04.1f  UTC %04.01f  VEL %04.01f  HS %04.01f | MAG %05.01f | STAT %05.01f\n",
				(float)gnss_lla_cnt/getElapsedTime(),
				(float)gnss_utc_cnt/getElapsedTime(),
				(float)gnss_vel_cnt/getElapsedTime(),
				(float)gnss_hs_cnt/getElapsedTime(),
				(float)mag_cnt/getElapsedTime(),
				(float)stat_p_cnt/getElapsedTime()
				);
	}
}
