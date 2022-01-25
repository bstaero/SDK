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
#include <thread>
#include <string>
#include <iostream>
#include <fstream>

#include "fp_test.h"
#include "fp_main.h"
#include "structs.h"
#include "test_handler.h"

#include "flight_plan.h"

enum FSM_TEST_STATE
{
    eINIT_STATE,
    eSEND_PAYLOAD_READY_ST,
    eZERO_GYROS_ST,
    eWAIT_CALIB_ST,
    eTAKE_OFF_ST,
    eWAITING_TAKEOFF_ST,
    eSEND_WP_ST,
    eCMD_AIRCRAFT_ST,
    eCMD_LAND_ST,
    eCMD_LANDING_ST,
    eFINISH_ST
};

static const double MULTIPLE_WP_DURATION = 1000; //sendMultipleWP test duration in secs.

static const double DEGREE_TO_METER_LATITUDE = 1.0 / 111319.4444;
static const double DEGREE_TO_METER_LONGITUDE = 1.213e-5;

static const double STEP_METER_INCREMENT = 5.0;

static float cmd_yaw = 0;

static FSM_TEST_STATE fsm_test_state = eINIT_STATE;

int updateCalibration() 
{
	static float end_time = 0.0;
	if(end_time == 0.0 && calibration_requested != UNKNOWN_SENSOR) {
		switch(calibration_requested) {
			case DYNAMIC_PRESSURE: end_time = getElapsedTime() + 2.0; break;
			case GYROSCOPE:        end_time = getElapsedTime() + 2.0; break;
			case MAGNETOMETER:     end_time = getElapsedTime() + 60.0; break;
		}
	}
	if(calibration_requested != UNKNOWN_SENSOR && getElapsedTime() < end_time)
    {       
        return 0;
    }		
	if(getElapsedTime() < end_time) 
    {		     
        return 1;
	} 
    else 
    {             
		calibration_requested = UNKNOWN_SENSOR;		
        return -1;
	}
	end_time = 0.0;
	waiting_on_calibrate = false;
}

static double send_wp_latitude = 0.0;
static double send_wp_longitude = 0.0;
static double send_wp_altitude = 0.0;

/***************************************************************************
	filename:	sendMultipleWP
	author:		Adolfo Diaz		
	purpose:	Implements issue test related with sending multiple WP
***************************************************************************/
size_t sendMultipleWP( int num_to_send )
{
//std::thread tRX_messages;
static int calibrated = 0;
static float waiting_tout;
static FlightPlan flight_plan;
static FlightPlanMap_t flight_plan_map;
Waypoint_t temp_waypoint;
Waypoint_t temp_waypoints[MAX_WAYPOINTS];
uint8_t num_points = 1;
static Command_t command;
static bool isFirstWPExecuted = false;
static size_t test_step = 0;
static size_t theoric_test_step = 0;
int local_step = 0;
double latitude_diff, longitude_diff, altitude_diff;

     /* Issue:
    ---------
    If we send multiple waypoints to the autopilot in less than 1.5 seconds between each,
    the autopilot becomes unresponsive, and does not execute the rest of the waypoints and 
    then becomes responsive again after a few seconds. Sometimes the autopilot becomes unresponsive 
    randomly and after a few seconds responds again. We have implemented a last waypoint retry 
    mechanism to resend the last waypoint every few seconds to mitigate this issue but overall 
    response to small adjustments becomes very slow.
    ----------
    */     

    switch (fsm_test_state)
    {
        case eINIT_STATE:            
            fsm_test_state = eSEND_PAYLOAD_READY_ST;
            waiting_tout = getElapsedTime();   

            break;

        case eSEND_PAYLOAD_READY_ST:
            if( getElapsedTime() > waiting_tout + 2  )
            {
                // validate payload interface is in ready state
                if ( payload_current_state  != PAYLOAD_CTRL_READY) {
                    printf("* Sending payload ready command mode ...");
                    if( !sendPayloadControlMode(PAYLOAD_CTRL_READY)) {
                        printf(" failed\n");
                        exitTest();   
                    }
                }
                // validate payload interface is in active state
                else if( validate_payload_control() ) {
                    printf(" done\n");
                    fsm_test_state = eZERO_GYROS_ST;
                } else {
                    printf(" failed\n");
                    exitTest();   
                }

                waiting_tout = getElapsedTime();
            }
            break;
        case eZERO_GYROS_ST:
            if( getElapsedTime() > waiting_tout + 1  )
            {
                printf("* Gyroscope Pressure Calibration Requested ...\n");
                sendCalibrate(GYROSCOPE);
                fsm_test_state = eWAIT_CALIB_ST;
                waiting_tout = getElapsedTime();                  
            }
            break;
        case eWAIT_CALIB_ST:
            if( getElapsedTime() > waiting_tout + 5 )
            {
                fsm_test_state = eTAKE_OFF_ST;
                waiting_tout = getElapsedTime();
            }
            break;
        case eTAKE_OFF_ST:            
            if( !validate_payload_control() ) {
                exitTest();            
                break;
            }

            printf("* Launch command ...\n");

            if (telemetry_system.flight_mode == FLIGHT_MODE_PREFLIGHT || telemetry_system.flight_mode == FLIGHT_MODE_LAUNCH) {
                // change to launch mode from preflight mode 
                if (telemetry_system.flight_mode == FLIGHT_MODE_PREFLIGHT) {

                    printf("** Switching to Launch Mode ...");
                    if (!setCheckCommandValue(CMD_FLIGHT_MODE, FLIGHT_MODE_LAUNCH, (uint8_t*)&telemetry_system.flight_mode, FLIGHT_MODE_LAUNCH, 1.0)) {
                        printf(" failed\n");
                        exitTest();
                        break;
                    } else
                        printf(" done\n");
                }

                // enable engine and launch 
                if (telemetry_system.flight_mode == FLIGHT_MODE_LAUNCH) {
                    printf("** Enabling Engine ...");
                    if (!setCheckCommandValue(CMD_ENGINE_KILL, 0, (uint8_t*)&telemetry_system.engine_on, 1, 1.0)) {
                        printf(" failed\n");
                        exitTest();
                        break;
                    }
                    else printf(" done\n");

                    printf("** Launching ...");
                    if (!setCheckCommandValue(CMD_LAUNCH, 0, (uint8_t*)&telemetry_system.flight_mode, FLIGHT_MODE_CLIMBOUT, 1.0)) {
                        printf(" failed\n");
                        exitTest();
                        break;
                    }
                    else printf(" launched\n");
                }
            }

            fsm_test_state = eWAITING_TAKEOFF_ST;
            waiting_tout = getElapsedTime();
            break;

        case eWAITING_TAKEOFF_ST:
            if (telemetry_system.flight_mode != FLIGHT_MODE_FLYING)
                waiting_tout = getElapsedTime();

            // after 5s of flying, start sending waypoints
            if( getElapsedTime() > waiting_tout + 5  )
            {                
                //show_telemetry = true;
                fsm_test_state = eSEND_WP_ST;
                waiting_tout = getElapsedTime();
            }
            break;

        case eSEND_WP_ST:  
            if( getElapsedTime() > waiting_tout + (stepTime/1000.0)  )
            {   
                // validate payload mode
                if( !validate_payload_control() ) {
                    exitTest();            
                    break;
                }

                latitude_diff = ( (send_wp_latitude - telemetry_position.latitude) / DEGREE_TO_METER_LATITUDE );
                longitude_diff = ( (send_wp_longitude - telemetry_position.longitude ) / DEGREE_TO_METER_LONGITUDE );
                altitude_diff = send_wp_altitude - telemetry_position.altitude;

                theoric_test_step++;

                 // Set the waypoint to send into the plan
                // if( ( abs( latitude_diff ) < 2.0 &&  abs( longitude_diff ) < 2.0 &&  abs( altitude_diff ) < 2.0) || test_step == 0 )
                // {                    
                //     // Increment way plan latitude for next iteration
                //     test_step++;               
                // }   

                local_step = test_step % 4;
                flight_plan.reset();    

                // Add a waypoint to the plan 
                temp_waypoint.num = 80;
                temp_waypoint.next = 80;
               
                switch( local_step )
                {
                    case 0:                        
                        temp_waypoint.latitude = 40.132347; // [deg]
                        temp_waypoint.longitude = -105.069237 + ( DEGREE_TO_METER_LONGITUDE * STEP_METER_INCREMENT ); // [deg]                          
                        break;
                    case 1:
                        temp_waypoint.latitude = 40.132347 + ( DEGREE_TO_METER_LATITUDE * STEP_METER_INCREMENT * -1 );    // [deg]
                        temp_waypoint.longitude = -105.069237 + ( DEGREE_TO_METER_LONGITUDE * STEP_METER_INCREMENT ); // [deg]  
                        break;
                    case 2:
                        temp_waypoint.latitude = 40.132347 + ( DEGREE_TO_METER_LATITUDE * STEP_METER_INCREMENT * -1 );    // [deg]
                        temp_waypoint.longitude = -105.069237; // [deg]  
                        break;
                    case 3:
                        temp_waypoint.latitude = 40.132347;    // [deg]
                        temp_waypoint.longitude = -105.069237; // [deg]  
                        break;
                    default:  
                        temp_waypoint.latitude = 40.132700;    // [deg]
                        temp_waypoint.longitude = -105.069237; // [deg]                      
                        break;              
                }   
                temp_waypoint.altitude = 1565.0;       // [m]
                temp_waypoint.radius = 0.0;            // [m]
                
                send_wp_latitude = temp_waypoint.latitude;
                send_wp_longitude = temp_waypoint.longitude;
                send_wp_altitude = temp_waypoint.altitude;

                flight_plan.add(&temp_waypoint, 1);

                // Get all waypoints
                num_points = flight_plan.getAll(temp_waypoints);

                memcpy(&flight_plan_map, flight_plan.getMap(), sizeof(FlightPlanMap_t));
                flight_plan_map.mode = ADD;

                // Sending vehicle to waypoint configured
                if( !sendFlightPlan((uint8_t *)temp_waypoints, num_points, &flight_plan_map) )
                    printf(" Command to send waypiont flightplan failed\n");
                else // Increment step for next iteration
                    test_step++;               
                      
                //printf("\n***  1.Sending WP --- step: %zu  th_step: %zu lat: %f  long: %f  alt: %f ***\n", test_step, theoric_test_step, temp_waypoint.latitude, temp_waypoint.longitude, temp_waypoint.altitude);    
                //printf("***  2.Telemetry WP --- lat: %f  long: %f  alt: %f ***\n", telemetry_position.latitude, telemetry_position.longitude, telemetry_position.altitude);    
                //printf("***  3.latitude_diff: %f  longitude_diff: %f  altitude_diff: %f  ***\n", latitude_diff, longitude_diff, altitude_diff);      
                printf("\n***  1.Sending WP --- step: %zu  th_step: %zu send_lat: %f  actual_lat: %f diff_lat:%f send_long: %f actual_long: %f diff_long:%f --- ***\n", test_step, theoric_test_step, temp_waypoint.latitude, telemetry_position.latitude, latitude_diff, temp_waypoint.longitude, telemetry_position.longitude, longitude_diff);    

                if( !isFirstWPExecuted )            
                    fsm_test_state = eCMD_AIRCRAFT_ST;

                if( num_to_send <= test_step )
                    fsm_test_state = eCMD_LAND_ST;

                waiting_tout = getElapsedTime();                                 
            }
            break;
        case eCMD_LAND_ST:
                // validate payload interface is in good state
                if( !validate_payload_control() )
                    break;

                printf("* Landing command ...\n");

                // is the system in flying mode, to trigger landing
                if (telemetry_system.flight_mode == FLIGHT_MODE_FLYING) {

                    printf("** Commanding landing mode ...");
                    if (!setCheckCommandValue(CMD_LAND, 1, (uint8_t*)&telemetry_system.flight_mode, FLIGHT_MODE_LANDING, 2.0)) {
                        printf(" failed\n");
                        exitTest();
                        break;
                    } else {
                        printf(" done\n");
                        printf("** Landing ...");
                    }
                } else if (telemetry_system.flight_mode != FLIGHT_MODE_LANDING) {
                    printf(" not in valid flight mode\n");
                    exitTest();
                }
                fsm_test_state = eCMD_LANDING_ST;
                waiting_tout = getElapsedTime();                                 
 
            break;

        case eCMD_LANDING_ST:
            if (telemetry_system.flight_mode == FLIGHT_MODE_LANDED) {
                printf(" LANDED !!\n");
                exitTest();
            }
            break;

        case eCMD_AIRCRAFT_ST:
            if( getElapsedTime() > waiting_tout + 2  )
            {                
                // validate payload mode
                if( !validate_payload_control() ) {
                    exitTest();            
                    break;
                }

                printf("************ eCMD_AIRCRAFT_ST ***********\n");
                printf("   * Sending vehicle to waypoint %d\n",  80);

                if( !setCommandValue(CMD_WAYPOINT, 80) )
                    printf(" Command to send to waypiont failed\n");
                else {
                    isFirstWPExecuted = true;
                    fsm_test_state = eSEND_WP_ST;
                }
               
                waiting_tout = getElapsedTime();
            }
            break;            
        default:                        
            break;
    }  

    // send heartbeat
    send_hb();

    /** Update control interval */
    static float last_ctrl = 0.0;
    float now = getElapsedTime();
    if (now - last_ctrl > CONTROL_INTERVAL) 
    {
        last_ctrl = now;
        /** Get State estimate for pitch/roll */
        //comm_handler->request(STATE_ESTIMATOR_PARAM, 0);

        /** Get agl value for raw control */
        comm_handler->request(SENSORS_AGL, 0);
    }

    return theoric_test_step;         
}

