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
#include <termios.h>

#include "test.h"
#include "main.h"
#include "structs.h"

#include "flight_plan.h"

// functional definitions
bool sendPayloadControlMode(PayloadControl_t control_value);
bool setCommandValue(bst::comms::CommandID_t id, float value);
bool sendCalibrate(SensorType_t sensor);

// packet for transmision
Packet tx_packet;

SystemInitialize_t system_init;

extern TelemetrySystem_t telemetry_system;
extern TelemetryControl_t telemetry_control;

volatile bool show_telemetry = false;

volatile bool received_reply = false;

// Payload state
PayloadControl_t payload_current_request = PAYLOAD_CTRL_OFF;
PayloadControl_t payload_current_state = PAYLOAD_CTRL_OFF;

// Commands
Command_t set_command;
volatile bool set_command_ack;

static float cmd_yaw = 0;

// Calibration
volatile bool waiting_on_calibrate = false;
extern volatile SensorType_t calibration_requested;

// for command line (terminal) input
struct termios initial_settings, new_settings;

void printTestHelp() {
    printf("Keys:\n");
    printf("  T   : Show telmetry\n");
    printf("\n");
    printf("  h   : Send payload heartbeat\n");
    printf("  r   : Send payload ready comannd \n");
    printf("  o   : Send payload off comannd \n");
    printf("  s   : Send payload shutdown comannd \n");
    printf("\n");
    printf("  z   : Zero gyros (required for launch)\n");
    printf("  t   : Send launch / land comannd \n");
    printf("\n");
    printf("  i   : Send vel_x=1\n");
    printf("  k   : Send vel_x=0 vel_y=0\n");
    printf("  ,   : Send vel_x=-1\n");
    printf("  j   : Send vel_y=-1\n");
    printf("  l   : Send vel_y=1\n");
    printf("\n");
    printf("  u   : Send vrate=1\n");
    printf("  g   : Send vrate=0\n");
    printf("  d   : Send vrate=-1\n");
    printf("  y   : Send heading += 90 degrees\n");
    printf("  Y   : Disable heading control\n");
    printf("\n");
	printf("  v   : Send command to go to velocity based flight mode\n");
    printf("  V   : Send command to go to position hold flight mode\n");
    printf("  f   : Send simple flight plan consisting of waypoint 80\n");
    printf("  F   : Send a four point flight plan consisting of waypoint 80\n");
    printf("  w   : Command aircraft to go to waypoint 80\n");
    printf("  D   : Delete flight plan consisting of waypoint 80\n");
    printf("  b   : Request Dubin's path information\n");
    printf("\n");
    printf("  p   : print this help\n");
}

bool inputAvailable() {
    // check for input on terminal
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);

    return (FD_ISSET(0, &fds));
}

void initializeTest() {

    // terminal settings to get input
    tcgetattr(0, &initial_settings);

    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 0;
    new_settings.c_cc[VTIME] = 0;

    tcsetattr(0, TCSANOW, &new_settings);

    // initialize system_init packet
    system_init.vehicle_type = PAYLOAD_NODE;
    system_init.serial_num = 0x1234;
}

#define HEARTBEAT_INTERVAL 0.5f // [s]
#define CONTROL_INTERVAL 0.05f // [s]
static float last_heartbeat = 0.0;
static float last_ctrl = 0.0;

void updateTest() {
    char input;

    Command_t command;

    uint8_t num_points = 0;
    Waypoint_t temp_waypoint;
    Waypoint_t temp_waypoints[MAX_WAYPOINTS];

    FlightPlanMap_t flight_plan_map;
    FlightPlan flight_plan;

    if (inputAvailable()) {
        input = getchar();

        if (input > 0) {
            switch (input) {

            case 'T':
								if(show_telemetry)
									show_telemetry = false;
								else show_telemetry = true;
								break;

            case 'a':
                printf("Setting automatic alt mode\n");
                command.id = CMD_ALT_MODE;
                command.value = ALT_MODE_AUTO;
                comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);
                break;

            case 'h':
                comm_handler->send(TELEMETRY_HEARTBEAT, NULL, 0, NULL);
                break;

            case 'r':
                printf("Sending payload ready command mode ...\n");
                sendPayloadControlMode(PAYLOAD_CTRL_READY);
                break;

            case 'o':
                printf("Sending payload off command ...\n");
                sendPayloadControlMode(PAYLOAD_CTRL_OFF);
                break;

            case 's':
                printf("Shutdown payload command ...\n");
                sendPayloadControlMode(PAYLOAD_CTRL_SHUTDOWN);
                break;

            case 'z':
                printf("Gyroscope Pressure Calibration Requested ...\n");
                sendCalibrate(GYROSCOPE);
                waiting_on_calibrate = true;
                break;

            case 't':
                printf("Launch / Land command ...\n");

                if (telemetry_system.flight_mode == FLIGHT_MODE_INVALID_MODE || telemetry_system.flight_mode == FLIGHT_MODE_LANDED) {
                    if (!setCommandValue(CMD_FLIGHT_MODE, FLIGHT_MODE_PREFLIGHT)) {
                        printf(" Not in Valid Mode\n");
                        break;
                    }
                }

                if (telemetry_system.flight_mode == FLIGHT_MODE_PREFLIGHT || telemetry_system.flight_mode == FLIGHT_MODE_LAUNCH) {
                    printf(" Launching vehicle\n");

                    if (telemetry_system.flight_mode == FLIGHT_MODE_PREFLIGHT) {
                        printf(" Switching to Launch Mode ...");
                        if (setCommandValue(CMD_FLIGHT_MODE, FLIGHT_MODE_LAUNCH)) {
                            printf(" done\n");
                        } else {
                            printf(" failed\n");
                            break;
                        }
                    }

                    while (telemetry_system.flight_mode != FLIGHT_MODE_LAUNCH) {
                        usleep(1000);
                        comm_handler->update();
                    }

                    printf(" Enabling Engine ...");
                    if (setCommandValue(CMD_ENGINE_KILL, 0)) {
                        printf(" done\n");
                    } else {
                        printf(" failed\n");
                        break;
                    }

                    printf(" Launching...");

                    if (setCommandValue(CMD_LAUNCH, 0)) {
                        printf(" done\n");
                    } else {
                        printf(" failed\n");
                        break;
                    }

                    break;
                }

                if (telemetry_system.flight_mode == FLIGHT_MODE_FLYING) {
                    printf(" Landing...");

                    if (setCommandValue(CMD_LAND, 1)) {
                        printf(" done\n");
                    } else {
                        printf(" failed\n");
                        break;
                    }

                    break;
                }

                printf(" Not in Valid Mode\n");

                break;

            case 'i':
				if (telemetry_control.nav_mode != NAV_PILOT_BODY && telemetry_control.nav_mode != NAV_PILOT_WORLD) {
					printf("YOU NEED TO CHANGE TO VELOCITY CONTROL MODE ('v' command): mode=%u\n", telemetry_control.nav_mode);
					break;
				}


                sendPayloadControlMode(PAYLOAD_CTRL_ACTIVE);

                printf("Sending velocity command\n");
                command.id = CMD_X_VEL;
                command.value = 1.0;
                comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);
                break;

			case 'k':
                if (telemetry_control.nav_mode != NAV_PILOT_BODY && telemetry_control.nav_mode != NAV_PILOT_WORLD) {
                    printf("YOU NEED TO CHANGE TO VELOCITY CONTROL MODE ('v' command): mode=%u\n", telemetry_control.nav_mode);
                    break;
                }

                sendPayloadControlMode(PAYLOAD_CTRL_ACTIVE);

                printf("Sending horizontal stop command\n");
                command.id = CMD_X_VEL;
                command.value = 0.0;
                comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);

                command.id = CMD_Y_VEL;
                command.value = 0.0;
                comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);

                break;

            case ',':
				if (telemetry_control.nav_mode != NAV_PILOT_BODY && telemetry_control.nav_mode != NAV_PILOT_WORLD) {
                    printf("YOU NEED TO CHANGE TO VELOCITY CONTROL MODE ('v' command): mode=%u\n", telemetry_control.nav_mode);
                    break;
                }

                sendPayloadControlMode(PAYLOAD_CTRL_ACTIVE);

                printf("Sending velocity command\n");
                command.id = CMD_X_VEL;
                command.value = -1.0;
                comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);
                break;

            case 'j':
				if (telemetry_control.nav_mode != NAV_PILOT_BODY && telemetry_control.nav_mode != NAV_PILOT_WORLD) {
                    printf("YOU NEED TO CHANGE TO VELOCITY CONTROL MODE ('v' command): mode=%u\n", telemetry_control.nav_mode);
                    break;
                }

                sendPayloadControlMode(PAYLOAD_CTRL_ACTIVE);

                printf("Sending velocity command\n");
                command.id = CMD_Y_VEL;
                command.value = -1.0;
                comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);
                break;

            case 'l':
				if (telemetry_control.nav_mode != NAV_PILOT_BODY && telemetry_control.nav_mode != NAV_PILOT_WORLD) {
                    printf("YOU NEED TO CHANGE TO VELOCITY CONTROL MODE ('v' command): mode=%u\n", telemetry_control.nav_mode);
                    break;
                }

                sendPayloadControlMode(PAYLOAD_CTRL_ACTIVE);

                printf("Sending velocity command\n");
                command.id = CMD_Y_VEL;
                command.value = 1.0;
                comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);
                break;

            case 'u':
                sendPayloadControlMode(PAYLOAD_CTRL_ACTIVE);
                command.id = CMD_ALT_MODE;
                command.value = ALT_MODE_RATE;
                comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);

                printf("Sending velocity command\n");
                command.id = CMD_VRATE;
                command.value = 1.0;
                comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);
                break;

            case 'g':
                sendPayloadControlMode(PAYLOAD_CTRL_ACTIVE);
                command.id = CMD_ALT_MODE;
                command.value = ALT_MODE_RATE;
                comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);

                printf("Sending velocity command\n");
                command.id = CMD_VRATE;
                command.value = 0.0;
                comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);
                break;

            case 'd':
                sendPayloadControlMode(PAYLOAD_CTRL_ACTIVE);
                command.id = CMD_ALT_MODE;
                command.value = ALT_MODE_RATE;
                comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);

                printf("Sending velocity command\n");
                command.id = CMD_VRATE;
                command.value = -1.0;
                comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);
                break;

            case 'y':
                sendPayloadControlMode(PAYLOAD_CTRL_ACTIVE);

                printf("Sending heading command\n");
                command.id = CMD_YAW;
								cmd_yaw = cmd_yaw+M_PI/2;
								if(cmd_yaw >= 2*M_PI) cmd_yaw = 0;
                command.value = cmd_yaw;
                comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);
                break;

            case 'Y':
                sendPayloadControlMode(PAYLOAD_CTRL_ACTIVE);

                printf("Sending heading command\n");
                command.id = CMD_YAW;
                command.value = NAN;
                comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);
                break;

			case 'v':
                sendPayloadControlMode(PAYLOAD_CTRL_ACTIVE);
                command.id = CMD_NAV_MODE;
                command.value = NAV_PILOT_WORLD;
                comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);

                printf("Sending velocity flight mode command\n");
                break;

            case 'V':
                sendPayloadControlMode(PAYLOAD_CTRL_ACTIVE);

                command.id = CMD_NAV_MODE;
                command.value = NAV_POS;
                comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);

                printf("Sending position hold flight mode command\n");
                break;



            // Send flight plan consisting of one orbit waypoint
            case 'f':
                printf("Sending a single waypoint\n");

                flight_plan.reset();

                // Add a waypoint to the plan
                temp_waypoint.num = 80;
                temp_waypoint.next = 80;
                temp_waypoint.latitude = 40.132700;    // [deg]
                temp_waypoint.longitude = -105.069231; // [deg]
                temp_waypoint.altitude = 1656.0;       // [m]
                temp_waypoint.radius = 0.0;            // [m]

                flight_plan.add(&temp_waypoint, 1);

                // Get all waypoints
                num_points = flight_plan.getAll(temp_waypoints);
                memcpy(&flight_plan_map, flight_plan.getMap(), sizeof(FlightPlanMap_t));
                flight_plan_map.mode = ADD;

                comm_handler->sendCommand(FLIGHT_PLAN, (uint8_t *)temp_waypoints, num_points, &flight_plan_map);
                break;

            case 'F':
                printf("Sending a four waypoint flight plan\n");

                flight_plan.reset();

                temp_waypoints[0].num = 80;
                temp_waypoints[0].next = 81;
                temp_waypoints[0].latitude = 40.133057;    // [deg]
                temp_waypoints[0].longitude = -105.069714; // [deg]
                temp_waypoints[0].altitude = 1636.0;       // [m]
                temp_waypoints[0].radius = 0.0;            // [m]

                temp_waypoints[1].num = 81;
                temp_waypoints[1].next = 82;
                temp_waypoints[1].latitude = 40.133057;    // [deg]
                temp_waypoints[1].longitude = -105.070368; // [deg]
                temp_waypoints[1].altitude = 1636.0;       // [m]
                temp_waypoints[1].radius = 0.0;            // [m]

                temp_waypoints[2].num = 82;
                temp_waypoints[2].next = 83;
                temp_waypoints[2].latitude = 40.132384;    // [deg]
                temp_waypoints[2].longitude = -105.070368; // [deg]
                temp_waypoints[2].altitude = 1636.0;       // [m]
                temp_waypoints[2].radius = 0.0;            // [m]

                temp_waypoints[3].num = 83;
                temp_waypoints[3].next = 80;
                temp_waypoints[3].latitude = 40.132384;    // [deg]
                temp_waypoints[3].longitude = -105.069714; // [deg]
                temp_waypoints[3].altitude = 1636.0;       // [m]
                temp_waypoints[3].radius = 0.0;            // [m]

                flight_plan.add(temp_waypoints, 4);

                // Get all waypoints
                num_points = flight_plan.getAll(temp_waypoints);
                memcpy(&flight_plan_map, flight_plan.getMap(), sizeof(FlightPlanMap_t));
                flight_plan_map.mode = ADD;

                comm_handler->sendCommand(FLIGHT_PLAN, (uint8_t *)temp_waypoints, num_points, &flight_plan_map);
                break;

            case 'D':
                printf("Delete waypoints 80,81,82,83\n");

                flight_plan.reset();

                temp_waypoints[0].num = 80;
                temp_waypoints[1].num = 81;
                temp_waypoints[2].num = 82;
                temp_waypoints[3].num = 83;

                flight_plan.add(temp_waypoints, 4);

                // Get all waypoints
                num_points = flight_plan.getAll(temp_waypoints);
                memcpy(&flight_plan_map, flight_plan.getMap(), sizeof(FlightPlanMap_t));
                flight_plan_map.mode = DELETE;

                comm_handler->sendCommand(FLIGHT_PLAN, (uint8_t *)temp_waypoints, num_points, &flight_plan_map);
                break;
							
            // Request Dubin's path 
            case 'b':
                printf("Requesting current Dubin's path\n");

								comm_handler->request(DUBIN_PATH, 0);
                break;

            // Send vehicle to waypoint 80
            case 'w':
                printf("Sending vehicle to waypoint 80\n");

                command.id = CMD_WAYPOINT;
                command.value = 80;
                comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);
                break;

            case 'p':
                printTestHelp();
                break;

            case 3: // <CTRL-C>
                    // allow flowthrough
            case 'q':
                printf("Keyboard caught exit signal ...\n");
                running = false;
                break;

            default:
                break;
            }
            input = 0;
        }
        else {
            clearerr(stdin);
        }
    }

    // send heartbeat

    float now = getElapsedTime();

    /** Update heartbeat */
    if (now - last_heartbeat > HEARTBEAT_INTERVAL) {
        last_heartbeat = now;
        comm_handler->send(TELEMETRY_HEARTBEAT, NULL, 0, NULL);
    }

    /** Update control interval */
    if (now - last_ctrl > CONTROL_INTERVAL) {
        last_ctrl = now;

				/** Get State estimate for pitch/roll */
				//comm_handler->request(STATE_ESTIMATOR_PARAM, 0);

				/** Get agl value for raw control */
				comm_handler->request(SENSORS_AGL, 0);
    }

}

void exitTest() {
    tcsetattr(0, TCSANOW, &initial_settings);
}

bool sendPayloadControlMode(PayloadControl_t control_value) {
    Command_t command;

    // maintain our requested state
    payload_current_request = control_value;

    command.id = CMD_PAYLOAD_CONTROL;
    command.value = control_value;
    comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&command, sizeof(Command_t), NULL);

    return true;
}

bool setCommandValue(bst::comms::CommandID_t id, float value) {
    set_command.id = id;
    set_command.value = value;
    set_command_ack = false;

    received_reply = false;

    comm_handler->sendCommand(CONTROL_COMMAND, (uint8_t *)&set_command, sizeof(Command_t), NULL);

    float sent_time = getElapsedTime();
    while (!received_reply && getElapsedTime() - sent_time < 2.0) {
        usleep(1000);
        comm_handler->update();
    }

    if (!received_reply) {
        return false;
    }

    received_reply = false;

    if (!set_command_ack) {
        return false;
    }

    return true;
}

bool sendCalibrate(SensorType_t sensor) {
    if (calibration_requested != UNKNOWN_SENSOR) {
        printf(" Failed: Pending request!\n");
        return false;
    }

    CalibrateSensor_t calibrate_pkt;

    switch (sensor) {
        case GYROSCOPE:
            break;
        default:
            printf(" Failed: Can only currently zero gyroscopes!\n");
            return false;
    }

    calibration_requested = sensor;

    calibrate_pkt.sensor = sensor;
    calibrate_pkt.state = SENT;

    printf("sending calibration packet\n");
    comm_handler->sendCommand(SENSORS_CALIBRATE, (uint8_t *)&calibrate_pkt, sizeof(CalibrateSensor_t), NULL);

    return true;
}
