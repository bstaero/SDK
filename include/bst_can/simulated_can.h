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

/*!
  \file   can.h
	\brief  Header for software in the loop simulation of CAN bus.

  \author Jack Elston
  \date
*/

#ifndef __SIMULATED_CAN_H_
#define __SIMULATED_CAN_H_

#include "bridge.h"

#ifdef __cplusplus

#include "comm_interface.h"

void setupSimulatedCAN(CommunicationsInterface * interface);

extern "C" {

#endif

uint8_t simulatedCANRead(void);
uint8_t simulatedCANWrite(uint8_t p, uint32_t id, void *data, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif
