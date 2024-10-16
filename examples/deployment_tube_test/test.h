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
#ifndef _TEST_H_
#define _TEST_H_

#include <inttypes.h>

#include "canpackets.h"
using namespace bst::comms::canpackets;

extern volatile bool display_telemetry;
extern volatile bool write_file;

extern CAN_DeploymentTube_t deployment_tube;
extern uint8_t new_deployment_tube_data;

void initializeTest(void);
void updateTest(void);
void exitTest(void);
void printTestHelp(void);

#endif
