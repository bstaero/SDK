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

#include "bst_protocol.h"
#define NUM_COMM_INTERFACES 1

extern bool big_endian;
extern bool running;

extern CommunicationsProtocol * comm_handler;

void printHelp();


void setupTime();

#ifdef __cplusplus
extern "C" {
#endif
float getElapsedTime();
#ifdef __cplusplus
}
#endif

#endif
