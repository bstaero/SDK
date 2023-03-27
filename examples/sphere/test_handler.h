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
#ifndef _TEST_HANDLER_H_
#define _TEST_HANDLER_H_

#include <inttypes.h>

typedef struct _Downstream_t {
	float time;
	double latitude;
	double longitude;
	float altitude;
	float q[4];
	float rh;
	float u;
	float v;
	float w;
	float speed_of_sound;
	float temperature;
} __attribute__ ((packed)) Downstream_t;

extern bool new_data;
extern Downstream_t downstream_pkt;

void receive(uint8_t type, void * data, uint16_t size, const void * parameter);
uint8_t receiveCommand(uint8_t type, void * data, uint16_t size, const void * parameter);
void receiveReply(uint8_t type, void * data, uint16_t size, bool ack, const void * parameter);
bool publish(uint8_t type, uint8_t param);

#endif
