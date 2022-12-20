/*=+--+=#=+--            SwiftPilot Autopilot Software            --+=#=+--+=#*\
|               Copyright (C) 2013 Black Swift Technologies LLC.               |
|                             All Rights Reserved.                             |

                                  Jack Elston                                   
|                          elstonj@blackswifttech.com                          |
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

uint8_t simulatedCANRead(uint8_t p);
uint8_t simulatedCANWrite(uint8_t p, uint32_t id, void *data, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif
