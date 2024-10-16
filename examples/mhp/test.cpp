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

#define VEHICLE_MULTIROTOR

#include "test.h"
#include "test_handler.h"

#include "main.h"

/*<---Global Variables---->*/
volatile bool display_telemetry = false;
volatile bool display_telemetry_timing = false;
volatile bool waiting_on_calibrate = false;
volatile bool waiting_on_orientation = false;
volatile bool waiting_on_mag = false;
volatile bool waiting_on_i_mag = false;
volatile bool write_file = false;

// for command line (terminal) input
struct termios initial_settings, new_settings;
/*<-End Global Variables-->*/

/*<---Local Functions----->*/
void updateCalibration(void);
void updateOrientation(void);
void updateMagCal(void);
void updateMagCurrentCal(void);
/*<-End Local Functions--->*/

void printTestHelp() {
	printf("Keys:\n");
	printf("  t   : Toggle telemetry display\n");
	printf("  i   : Toggle telemetry timing display\n");
	printf("\n");
	printf("  d   : Request dynamic pressure calibration\n");
	printf("  g   : Request gyroscope calibration\n");
	printf("  m   : Request magnetometer calibration\n");
	printf("  M   : Send magnetometer calibration\n");
	printf("  c   : Request magnetometer current calibration\n");
	printf("  C   : Send magnetometer current calibration\n");
	printf("  O   : Command magnetometer calibration\n");
	printf("  H   : Command humidity recondition\n");
	printf("\n");
	printf("  u   : Request imu orientation\n");
	printf("  U   : Set imu orientation with mount above probe\n");
	printf("  B   : Set imu orientation with mount below probe\n");
	printf("  n   : Request GNSS orientation\n");
	printf("  N   : Set GNSS orientation with cable facing back\n");
	printf("\n");
	printf("  I   : Send simulated current data\n");
	printf("\n");
	printf("  s   : Request serial number and comms revision\n");
	printf("\n");
	printf("  p   : Print this help\n");
}

bool inputAvailable()  
{
	// check for input on terminal
	struct timeval tv;
	fd_set fds;
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds);
	select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);

	return (FD_ISSET(0, &fds));
}


void initializeTest() {

	// terminal settings to get input
	tcgetattr(0,&initial_settings);

	new_settings = initial_settings;
	new_settings.c_lflag &= ~ICANON;
	new_settings.c_lflag &= ~ECHO;
	new_settings.c_lflag &= ~ISIG;
	new_settings.c_cc[VMIN] = 0;
	new_settings.c_cc[VTIME] = 0;

	tcsetattr(0, TCSANOW, &new_settings);
}

void updateTest() {
	static bool first_run = true;
	if(first_run) {
		printTestHelp();
		first_run = false;
	}

	AxisMapping_t temp_axis_mapping;
	int temp_int = 0;

	if(waiting_on_calibrate) updateCalibration();
	if(waiting_on_orientation) updateOrientation();
	if(waiting_on_mag) updateMagCal();
	if(waiting_on_i_mag) updateMagCurrentCal();

	char input; 

	if( inputAvailable() ) {
		input = getchar();

		if(input > 0) {
			switch(input) {

				case 't':
					display_telemetry_timing = false;
					display_telemetry? display_telemetry=false: display_telemetry=true;
					break;

				case 'i':
					display_telemetry = false;
					display_telemetry_timing? display_telemetry_timing=false: display_telemetry_timing=true;
					break;

				case 'd':
					sendCalibrate(DYNAMIC_PRESSURE);
					waiting_on_calibrate = true;
					printf("Dynamic Pressure Calibration Requested.. ");
					fflush(stdout);
					break;

				case 'g':
					sendCalibrate(GYROSCOPE);
					waiting_on_calibrate = true;
					printf("Gyroscope Pressure Calibration Requested.. ");
					fflush(stdout);
					break;

				case 'O':
					sendCalibrate(MAGNETOMETER);
					waiting_on_calibrate = true;
					printf("Magnetometer Calibration Requested.. ");
					fflush(stdout);
					break;

				case 'M':
					if(waiting_on_mag) break;
					waiting_on_mag = true;

					sendMagCalibraton();
					printf("Magnetometer Calibration Sent.. ");
					fflush(stdout);
					break;

				case 'm':
					requestMagCalibration();
					break;

				case 'C':
					if(waiting_on_i_mag) break;
					waiting_on_i_mag = true;

					sendMagCurrentCalibraton();
					printf("Magnetometer Current Calibration Sent.. ");
					fflush(stdout);
					break;

				case 'c':
					requestMagCurrentCalibration();
					break;

				case 'H':
					sendCalibrate(HUMIDITY);
					waiting_on_calibrate = true;
					printf("Humidity Recondition Requested.. ");
					fflush(stdout);
					break;

				case 'u':
					requestOrientation(SENSORS_BOARD_ORIENTATION);
					break;

				case 'U':
					if(waiting_on_orientation) break;
					waiting_on_orientation = true;

					// mount pointing up - aeropods
					//temp_axis_mapping.axis[0] = -2;
					//temp_axis_mapping.axis[1] = -1;
					//temp_axis_mapping.axis[2] = -3;

					// v06a (Altius)
					temp_axis_mapping.axis[0] = 1;
					temp_axis_mapping.axis[1] = 2;
					temp_axis_mapping.axis[2] = 3;

					printf("Setting IMU orietation to [%i,%i,%i].. ",
							temp_axis_mapping.axis[0],
							temp_axis_mapping.axis[1],
							temp_axis_mapping.axis[2]);
					fflush(stdout);

					setOrientation(SENSORS_BOARD_ORIENTATION, &temp_axis_mapping);
					break;

				case 'B':
					if(waiting_on_orientation) break;
					waiting_on_orientation = true;
					
					// mount pointing down
					temp_axis_mapping.axis[0] = -2;
					temp_axis_mapping.axis[1] = 1;
					temp_axis_mapping.axis[2] = 3;

					printf("Setting IMU orietation to [%i,%i,%i].. ",
							temp_axis_mapping.axis[0],
							temp_axis_mapping.axis[1],
							temp_axis_mapping.axis[2]);
					fflush(stdout);

					setOrientation(SENSORS_BOARD_ORIENTATION, &temp_axis_mapping);
					break;

				case 'n':
					requestOrientation(SENSORS_GNSS_ORIENTATION);
					break;

				case 'N':
					if(waiting_on_orientation) break;
					waiting_on_orientation = true;

					// board silk
					//temp_axis_mapping.axis[0] = 1;
					//temp_axis_mapping.axis[1] = 2;
					//temp_axis_mapping.axis[2] = 3;

					// aeropod mount
					//temp_axis_mapping.axis[0] = 2;
					//temp_axis_mapping.axis[1] = -1;
					//temp_axis_mapping.axis[2] = 3;
					
					// cable to the back
					temp_axis_mapping.axis[0] = -2;
					temp_axis_mapping.axis[1] = 1;
					temp_axis_mapping.axis[2] = 3;


					printf("Setting GNSS orietation to [%i,%i,%i].. ",
							temp_axis_mapping.axis[0],
							temp_axis_mapping.axis[1],
							temp_axis_mapping.axis[2]);
					fflush(stdout);

					setOrientation(SENSORS_GNSS_ORIENTATION, &temp_axis_mapping);
					break;

				case 'I':
					sendCurrent();
					break;

				case 's':
					requestPowerOn();
					break;

				case 'p':
					printTestHelp();
					break;

				case 3: // <CTRL-C> 
				case 'q':
					printf("Keyboard caught exit signal ...\n");
					running = false;
					break;

				default:
					break;
			}
			input = 0;
		} else {
			clearerr(stdin);
		}
	}

}

void updateCalibration() {
	static float end_time = 0.0;
	if(end_time == 0.0 && calibration_requested != UNKNOWN_SENSOR) {
		switch(calibration_requested) {
			case DYNAMIC_PRESSURE: end_time = getElapsedTime() + 2.0; break;
			case GYROSCOPE:        end_time = getElapsedTime() + 2.0; break;
			case MAGNETOMETER:     end_time = getElapsedTime() + 60.0; break;
			case HUMIDITY:         end_time = getElapsedTime() + 260.0; break;
		}
	}

	if(calibration_requested != UNKNOWN_SENSOR && getElapsedTime() < end_time)
		return;

	if(getElapsedTime() < end_time) {
		printf("SUCCESS\n");
	} else {
		calibration_requested = UNKNOWN_SENSOR;
		printf("FAILED\n");
	}

	end_time = 0.0;
	waiting_on_calibrate = false;
}

void updateOrientation() {
	static float end_time = 0.0;
	if(end_time == 0.0 && orientation_requested != 0) {
		end_time = getElapsedTime() + 2.0;;
	}

	if(orientation_requested != 0 && getElapsedTime() < end_time)
		return;

	if(getElapsedTime() < end_time && orientation_action == PKT_ACTION_ACK) {
		printf("SUCCESS\n");
	} else {
		calibration_requested = UNKNOWN_SENSOR;
		printf("FAILED\n");
	}

	orientation_requested = (PacketTypes_t)0;

	end_time = 0.0;
	waiting_on_orientation = false;
}

void updateMagCal() {
	static float end_time = 0.0;
	if(end_time == 0.0 && mag_cal_requested != 0) {
		end_time = getElapsedTime() + 2.0;;
	}

	if(mag_cal_requested != 0 && getElapsedTime() < end_time)
		return;

	if(getElapsedTime() < end_time && mag_cal_action == PKT_ACTION_ACK) {
		printf("SUCCESS\n");
	} else {
		printf("FAILED\n");
	}

	mag_cal_requested = (PacketTypes_t)0;

	end_time = 0.0;
	waiting_on_mag = false;
}

void updateMagCurrentCal() {
	static float end_time = 0.0;
	if(end_time == 0.0 && mag_i_cal_requested != 0) {
		end_time = getElapsedTime() + 2.0;;
	}

	if(mag_i_cal_requested != 0 && getElapsedTime() < end_time)
		return;

	if(getElapsedTime() < end_time && mag_i_cal_action == PKT_ACTION_ACK) {
		printf("SUCCESS\n");
	} else {
		printf("FAILED\n");
	}

	mag_i_cal_requested = (PacketTypes_t)0;

	end_time = 0.0;
	waiting_on_i_mag = false;
}


void exitTest() {
	tcsetattr(0, TCSANOW, &initial_settings);
}
