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
#include "structs.h"
#include "flight_plan.h"


void receive(uint8_t type, void * data, uint16_t size, const void * parameter);
uint8_t receiveCommand(uint8_t type, void * data, uint16_t size, const void * parameter);
void receiveReply(uint8_t type, void * data, uint16_t size, bool ack, const void * parameter);
bool publish(uint8_t type, uint8_t param);


// Setting / Configuration
#define CMD_ACK_TIMEOUT    0.02f  // [s] how long to wait for message ACK
#define CMD_RETRIES        1      // number of retires on command
#define HEARTBEAT_INTERVAL 0.5f   // [s] time between heartbeat
#define CONTROL_INTERVAL   0.05f  // [s]

// functional definitions
bool waitForACK();
bool setCommandValue(bst::comms::CommandID_t id, float value);
bool setCheckCommandValue(uint8_t id, int value, uint8_t* check, uint8_t check_value, float timeout );

bool sendFlightPlan(uint8_t *temp_waypoints, uint8_t num_points, FlightPlanMap_t *flight_plan_map);
bool sendPayloadControlMode(PayloadControl_t control_value);
bool sendCalibrate(SensorType_t sensor);
void send_hb();


bool validate_alt_rate_mode();
bool validate_payload_control();
bool validate_nav_mode();

// Calibration
extern volatile bool waiting_on_calibrate;
extern volatile SensorType_t calibration_requested;

// System state
extern SystemInitialize_t  system_init;
extern TelemetrySystem_t   telemetry_system;
extern TelemetryControl_t  telemetry_control;
extern TelemetryPosition_t telemetry_position;
extern PayloadControl_t    payload_current_state;

//extern bool received_reply;
extern Command_t     set_command;
//extern bool set_command_ack;

extern bool show_telemetry;

#endif
