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
#include <string.h>

#include "test_handler.h"

#include "test.h"
#include "main.h"
#include "structs.h"

/*<---Global Variables---->*/
Packet rx_packet;
Packet tx_packet;

LDCR_t           ldcr_data;

uint32_t product_cnt = 0;
uint32_t sensors_cnt = 0;

float last_print = 0;
/*<-End Global Variables-->*/

/*<---Local Functions----->*/
void handlePacket(uint8_t type, uint8_t action, const void * data, uint16_t size);
void printLDCRValues(void);

bool isValid(uint8_t data);
/*<-End Local Functions--->*/

bool updateCommunications(void) {

	uint8_t data;
	bool retval = false;
	rx_packet.setAddressing(false);

	while(readByte(&data)) {
		retval = true;
#if 0 // probably should switch to a packet based structure
		if(rx_packet.isValid(data)) {
			handlePacket(rx_packet.getType(), rx_packet.getAction(), rx_packet.getDataPtr(), rx_packet.getDataSize());
#else
		if(isValid(data)) {
			handlePacket(PAYLOAD_LDCR, PKT_ACTION_STATUS, &ldcr_data, sizeof(LDCR_t));
#endif
			break;
		}
	}

	return retval;
}

void handlePacket(uint8_t type, uint8_t action, const void * data, uint16_t size) 
{
	//printf("handlePacket: type=%u\n", type);

	PowerOn_t * power_on_data_ptr;

	switch(type) {

		case PAYLOAD_LDCR:
			product_cnt++;

			//memcpy(&ldcr_data,data,sizeof(LDCR_t));

			printLDCRValues();

			break;

						/* ERRORS */
		default:
		case INVALID_PACKET:
						break;
	}
}

bool checkFletcher16(const uint8_t * const data, uint8_t size) {
	uint16_t sum1 = 0;
	uint16_t sum2 = 0;

	for( int i = 0; i < size; i++ ) {
		sum1 = (sum1 + data[i]) % 255;
		sum2 = (sum2 + sum1) % 255;
	}

	return ((sum2 << 8) | sum1) == 0;
}

bool isValid(uint8_t data) {
	static uint16_t pkt_ptr = 0;
	static LDCR_t tmp_ldcr;
	static uint8_t * data_ptr = (uint8_t *)&tmp_ldcr;

	data_ptr[pkt_ptr] = data;

	switch(pkt_ptr)
	{
		case 0:
			if(tmp_ldcr.header[0] != 'U') {
				printf("ERROR - Bad header byte\n");
				pkt_ptr=0;
			} else {
				pkt_ptr++;
			}
			break;
		case 1:
			if(tmp_ldcr.header[1] != '$') {
				printf("ERROR - Bad header byte\n");
				pkt_ptr=0;
			} else {
				pkt_ptr++;
			}
			break;
		case (sizeof(LDCR_t)-1):
			if(checkFletcher16((uint8_t *)&tmp_ldcr,sizeof(LDCR_t))) {
				pkt_ptr = 0;
				memcpy(&ldcr_data,&tmp_ldcr,sizeof(LDCR_t));
				return true;
			} else {
				printf("ERROR - Invalid checksum\n");
			}

			pkt_ptr = 0;
			break;
		default:
			pkt_ptr++;
			break;
	}

	return false;

}

void requestPowerOn(void) {
		printf("0x%04x %c 0x%08x %u 0x%04x\n\r",
				ldcr_data.serial_number,
				ldcr_data.hw_revision,
				ldcr_data.sw_revision,
				ldcr_data.platform_type,
				ldcr_data.platform_serial
				);
}

#define ADC_raw2volt (3.3f/0xFFF) // reference is 3.3V

void printLDCRValues(void) {
	static uint32_t num_readings = 0;
	num_readings++;
	static uint8_t counter=0;

	if(ldcr_data.calibration_state == 25) {
		counter ++;
	} else {
		counter = 0;
	}

	if(display_telemetry && counter > 3)
		//printf("0x%04x %c 0x%08x %u 0x%04x %07u %02u %+010i %+010i %014lu %014lu [ %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f | %0.2f ]\n\r",
				//ldcr_data.serial_number,
				//ldcr_data.hw_revision,
				//ldcr_data.sw_revision,
				//ldcr_data.platform_type,
				//ldcr_data.platform_serial,
		//printf("%07u %02u %+010i %+010i %016lu %016lu [ %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f | %0.2f ] %0.2f\n\r",
		printf("%u,%u,%i,%i,%lu,%lu,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f\n\r",
				ldcr_data.system_time,
				ldcr_data.calibration_state,
				ldcr_data.sum_data[0],
				ldcr_data.sum_data[1],
				ldcr_data.sum_of_squares[0],
				ldcr_data.sum_of_squares[1],
				(float)ldcr_data.thermistor[0]/15.0 * ADC_raw2volt,
				(float)ldcr_data.thermistor[1]/15.0 * ADC_raw2volt,
				(float)ldcr_data.thermistor[2]/15.0 * ADC_raw2volt,
				(float)ldcr_data.thermistor[3]/15.0 * ADC_raw2volt,
				(float)ldcr_data.thermistor[4]/15.0 * ADC_raw2volt,
				(float)ldcr_data.thermistor[5]/15.0 * ADC_raw2volt,
				(float)ldcr_data.thermistor[6]/15.0 * ADC_raw2volt,
				(float)ldcr_data.thermistor[7]/15.0 * ADC_raw2volt,
				(float)ldcr_data.thermistor_ref/15.0 * ADC_raw2volt,
				(float)num_readings/getElapsedTime()
				//uint16_t week;
				//uint8_t hour;
				//uint8_t minute;
				//float seconds;
				//double latitude;  // [deg]
				//double longitude;  // [deg]
				//float altitude;  // [m]
				//float agl;  // [m]
				//float roll;  // [rad]
				//float pitch;  // [rad]
					);

				if(write_file) {
							writeFile((uint8_t*)&ldcr_data,sizeof(LDCR_t));
				}
}
