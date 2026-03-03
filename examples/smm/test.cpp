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

#include "flight_plan.h"

// variables
bool show_telemetry = false;

// functional definitions
bool sendPayloadData(uint8_t channel, uint8_t * data, uint8_t size);

// packet for transmision
Packet              tx_packet;

extern TelemetryOrientation_t telemetry_orientation;
extern TelemetryPosition_t    telemetry_position;
extern TelemetryPressure_t    telemetry_pressure;
extern TelemetrySystem_t      telemetry_system;
extern TelemetryControl_t     telemetry_control;

UserPayload_t                 tx_payload;

LDCRCommand_t ldcr_command;
LDCRTelemetry_t ldcr_telemetry;

void printTestHelp() {
	printf("Keys:\n");
	printf("  t   : Toggle Telemetry Display\n");
	printf("\n");
	printf("  p   : print this help\n");
}

void updateTest() {
	char input; 

	Command_t command;

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
			input = 0;
		} else {
			clearerr(stdin);
		}
	}
	// show telemetry from UAS

	static float last_telemetry = 0;
	if(getElapsedTime() - last_telemetry > 1.0) {
		last_telemetry = getElapsedTime();

		ldcr_telemetry.sensor_status = ldcr_command.sensor_command;
		ldcr_telemetry.time = getElapsedTime();
		ldcr_telemetry.latitude = (float)telemetry_position.latitude / 1e16;
		ldcr_telemetry.longitude = (float)telemetry_position.longitude / 1e16;
		ldcr_telemetry.altitude = (float)telemetry_position.altitude / 1000;
		ldcr_telemetry.height = (float)telemetry_position.height / 100;
		ldcr_telemetry.ndvi = 0.8;
		ldcr_telemetry.ground_temperature = 22.1;
		ldcr_telemetry.brightness_temperature = 300.2;

		sendPayloadData(1, (uint8_t*)&ldcr_telemetry, sizeof(LDCRTelemetry_t));

		if(show_telemetry) {

			printf("%04u %03u:%02u:%05.02f | ",
					telemetry_system.week,
					telemetry_system.hour,
					telemetry_system.minute,
					(float)telemetry_system.milliseconds / 1000
					);

			printf("lla: (%+06.02f,%+07.02f) %06.01f %05.01f | ",
					(float)telemetry_position.latitude / 1e16,
					(float)telemetry_position.longitude / 1e16,
					(float)telemetry_position.altitude / 1000,
					(float)telemetry_position.height / 100
					);

			float q_f[4];
			q_f[0] = (float)telemetry_orientation.q[0] / 10000;
			q_f[1] = (float)telemetry_orientation.q[0] / 10000;
			q_f[2] = (float)telemetry_orientation.q[0] / 10000;
			q_f[3] = (float)telemetry_orientation.q[0] / 10000;

			printf("<%+05.02f,%+05.02f,%+05.02f>\n",
					quat_to_roll(q_f) * 180 / M_PI,
					quat_to_pitch(q_f) * 180 / M_PI,
					quat_to_yaw(q_f) * 180 / M_PI
					);
		}
	}
}

void publishPayloadCommand(uint8_t channel) {
	uint8_t ptr = 0;
	if(channel + PAYLOAD_DATA_CHANNEL_0 > PAYLOAD_DATA_CHANNEL_7) return;

	bzero(tx_payload.buffer,64);

	switch(channel + PAYLOAD_DATA_CHANNEL_0) {
		case PAYLOAD_DATA_CHANNEL_0:
			memcpy(tx_payload.buffer,(uint8_t*)&(ldcr_command),sizeof(LDCRCommand_t));
			comm_handler->send(PAYLOAD_DATA_CHANNEL_0 + channel, (uint8_t *)&tx_payload, sizeof(UserPayload_t), NULL);
			break;

		case PAYLOAD_DATA_CHANNEL_1:
		case PAYLOAD_DATA_CHANNEL_2:
		case PAYLOAD_DATA_CHANNEL_3:
		case PAYLOAD_DATA_CHANNEL_4:
		case PAYLOAD_DATA_CHANNEL_5:
		case PAYLOAD_DATA_CHANNEL_6:
		case PAYLOAD_DATA_CHANNEL_7:
			break;
	}
}

bool setPayloadCommand(uint8_t channel, UserPayload_t * data) {
	uint8_t ptr = 0;
	if(channel + PAYLOAD_DATA_CHANNEL_0 > PAYLOAD_DATA_CHANNEL_7) return false;

	LDCRCommand_t * tmp_ptr = (LDCRCommand_t*)data->buffer;

	switch(channel + PAYLOAD_DATA_CHANNEL_0) {
		case PAYLOAD_DATA_CHANNEL_0:
			if(tmp_ptr->sensor_command >= LDCR_STATUS_INVALID) return false;

			memcpy((uint8_t*)&(ldcr_command),data->buffer,sizeof(LDCRCommand_t));
			printf("Setting LDCR status to %u\n", tmp_ptr->sensor_command);
			return true;
			break;

		case PAYLOAD_DATA_CHANNEL_1:
		case PAYLOAD_DATA_CHANNEL_2:
		case PAYLOAD_DATA_CHANNEL_3:
		case PAYLOAD_DATA_CHANNEL_4:
		case PAYLOAD_DATA_CHANNEL_5:
		case PAYLOAD_DATA_CHANNEL_6:
		case PAYLOAD_DATA_CHANNEL_7:
			break;
	}

	return false;
}


bool sendPayloadData(uint8_t channel, uint8_t * data, uint8_t size) {
	uint8_t ptr = 0;
	if(channel + PAYLOAD_DATA_CHANNEL_0 > PAYLOAD_DATA_CHANNEL_7) return false;

	bzero(tx_payload.buffer,64);

	while(ptr < size) {
		if(size-ptr > 64) tx_payload.size = 64;
		else tx_payload.size = size-ptr;

		memcpy(tx_payload.buffer,&(data[ptr]),tx_payload.size);

		comm_handler->send(PAYLOAD_DATA_CHANNEL_0 + channel, (uint8_t *)&tx_payload, sizeof(UserPayload_t), NULL);

		ptr += tx_payload.size;
		tx_payload.size = 0;
		for(uint8_t i=0; i<64; i++) tx_payload.buffer[i] = 0;
	}

	return true;
}
