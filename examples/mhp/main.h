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
#ifndef _MAIN_H_
#define _MAIN_H_

#include "bst_packet.h"
#include "debug.h"

extern bool big_endian;
extern bool running;

bool readByte(uint8_t * data);
bool writeBytes(uint8_t * data, uint16_t num);

bool writeFile(uint8_t * data, uint16_t num);

void printHelp();

void setupTime();

extern "C" {
	float getElapsedTime();
}

#endif
