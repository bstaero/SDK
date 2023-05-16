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
#include "bst_packet.h"

extern volatile SensorType_t calibration_requested;
extern volatile PacketTypes_t orientation_requested;
extern volatile PacketTypes_t mag_cal_requested;
extern volatile PacketAction_t orientation_action;
extern volatile PacketAction_t mag_cal_action;

void sendCalibrate(SensorType_t sensor);
void sendMagCalibraton(void);
void requestPowerOn(void);
void requestOrientation(PacketTypes_t type);
void requestMagCalibration(void);
void setOrientation(PacketTypes_t type, AxisMapping_t * axis_mapping);
bool updateCommunications(void);

#endif
