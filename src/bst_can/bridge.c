/*=+--+=#=+--            SwiftPilot Autopilot Software            --+=#=+--+=#*\
|               Copyright (C) 2012 Black Swift Technologies LLC.               |
|                             All Rights Reserved.                             |

     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License version 2 as
     published by the Free Software Foundation.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                  Jack Elston                                   
|                          elstonj@blackswifttech.com                          |
|                                                                              |
\*=+--+=#=+--                 --+=#=+--+=#=+--                    --+=#=+--+=#*/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <float.h>
#include <string.h>
#include "bridge.h"

#include "canpackets.h"

#ifdef __cplusplus
using namespace bst::comms::canpackets;
#endif

#if ! defined ARCH_stm32f1 && ! defined STM32F413xx && ! defined STM32F405xx
  #include "helper_functions.h"
  #include "simulated_can.h"
#endif

#include "debug.h"

#if defined ARCH_stm32f4 || defined ARCH_stm32f1 || defined STM32F413xx || defined STM32F405xx
  #if ! defined STM32F413xx && ! defined STM32F405xx
    #include "can.h"
    #include "led.h" // DEBUG
  #else
uint8_t CAN_Write(uint8_t p, uint32_t id, void *data, uint8_t size) {
	return 0;
}
  #endif

  #if defined ARCH_stm32f1 || defined STM32F413xx || defined STM32F405xx
    uint8_t checkFletcher16(uint8_t * data, uint8_t size);
    void setFletcher16 (uint8_t * data, uint8_t size);
  #endif
#else
	uint8_t CAN_Read(uint8_t p) {return simulatedCANRead();}
	uint8_t CAN_Write(uint8_t p, uint32_t id, void *data, uint8_t size) {
		return simulatedCANWrite(p, id, data, size);
	}

#endif

#if defined ARCH_stm32f4 || defined IMPLEMENTATION_xplane // FIXME -- remove
  #include "main.h"
#endif

#if defined BOARD_core && defined IMPLEMENTATION_firmware
 #include "uart.h"
 #include "clock.h"
#endif

#ifdef BOARD_pro_arbiter
 #include "pro_arbiter.h"
#endif

#if defined _SP_ACTUATOR || defined _SP_ACTUATOR_HACKHD || defined _SP_ACTUATOR_A6000
 #include "dip.h"
#endif

/** @addtogroup Source
 * @{
 */ 
#ifndef ARCH_stm32f1
extern uint32_t gps_count;
extern uint32_t imu_count;
extern uint32_t pressure_count;
extern uint32_t pwm_in_count;
#endif

extern uint8_t can_pressure_sensor; // implemented elsewhere
extern uint8_t can_imu_sensor; // implemented elsewhere
extern uint8_t can_mag_sensor; // implemented elsewhere
extern uint8_t can_gps_sensor; // implemented elsewhere

extern uint8_t can_pwm_in; // implemented elsewhere

void updateActuatorValues(uint16_t * values); // implemented elsewhere
void updatePWMIn(float system_time, uint16_t * usec); // implemented elsewhere

void updateGPS(
		float system_time, uint16_t week, uint8_t hour, uint8_t minute, float seconds,
		double latitude, double longitude, float altitude,
		float vel_n, float vel_e, float vel_d,
		float course, float speed,
		float pdop, uint8_t satellites, uint8_t fix_type);

void updateGPSRTCM(float system_time, uint8_t size, uint8_t * data);

void updateGPSSVIN(
		uint32_t time_elapsed,
		uint32_t time_minimum,
		float accuracy,
		float accuracy_minimum,
		uint8_t flags);

// ---
// All of the values in the update functions should be referenced to
// the autopilot frame, defined by the PCB
// ---
void updateAccelerometer(float system_time,
		float ax, float ay, float az); // [m/s^2]

void updateGyroscope(float system_time,
		float gx, float gy, float gz); // [rad/s]

void updateMagnetometer(float system_time,
		float mx, float my, float mz); // [uT]

void updateIMU(float system_time, 
		float ax, float ay, float az, 
		float gx, float gy, float gz, 
		float mx, float my, float mz);

void updateDynamicPressure(float system_time,
		float pressure, float temperature); // [hPa, deg C]

void updateStaticPressure(float system_time,
		float pressure, float temperature); // [hPa, deg C]

void updateMHP(float system_time,
		float static_pressure,
		float dynamic_pressure[5],
		float temperature,
		float humidity,
		float gyroscope[3],
		float accelerometer[3],
		float magnetometer[3],
		float alpha,
		float beta); // [hPa, hPa[5], deg C, %, rad, rad]

void updateAGL(float system_time,
		float distance); // [m]

void updateProximity(float system_time,
		float distance); // [m]

void updateTemperature(float system_time,
		float temperature); // [deg C]

void updateHumidity(float system_time,
		float humidity); // [%]

void updateSupply(float system_time,
		float voltage, float current, float coulomb_count, float temperature); // [V, A, mAh, deg c]

void updateADSB(float system_time,
		uint32_t icao_address,
		double latitude,
		double longitude,
		uint8_t altitude_type,
		float altitude,
		float heading,
		float horizontal_velocity,
		float vertical_velocity,
		char callsign[9],
		uint8_t emitter_type,
		uint8_t tslc,
		uint16_t flags,
		uint16_t squawk);

/** @addtogroup Low_Level
 * @{
 */ 

/** @addtogroup BRIDGE
 * @{
 */	 

/** @defgroup BRIDGE 
 * @brief BRIDGE setup
 * @{
 */ 

/** @defgroup BRIDGE_Private_Defines
 * @{
 */ 
#define BRIDGE_START_BYTE '@'	
#define BRIDGE_PACKET_SIZE 128

#if 0 // CAN ONLY USE WITH "FIXED" CAN BOARDS
#define BRIDGE_BUFFER_PREAMBLE\
	static uint8_t cnt = 0;\
\
	if(cnt + size > pkt_size) {\
		cnt = 0u; buffer[0] = 0;\
	}\
\
	memcpy(buffer + cnt, byte, size);\
	cnt += size;\
\
	if (buffer[0] == BRIDGE_START_BYTE) {\
		if (cnt == pkt_size) {\
			if (checkFletcher16(buffer, pkt_size)) {

#define BRIDGE_BUFFER_CONCLUSION\
			} else {\
				pmesg(VERBOSE_ERROR, "%s: ERROR\r\n",function_name);\
				BRIDGE_pktDrops++;\
			}\
			cnt = 0u; buffer[0] = 0;\
		}\
	} else {\
		cnt = 0u; buffer[0] = 0;\
	}
#else
#define BRIDGE_BUFFER_PREAMBLE\
	static uint8_t cnt = 0;\
\
	uint8_t i;\
	for(i=0; i<size; i++) {\
\
		if(cnt > pkt_size) {\
			cnt = 0u; buffer[0] = 0;\
		}\
\
		buffer[cnt++] = byte[i];\
\
		if (buffer[0] == BRIDGE_START_BYTE) {\
			if (cnt == pkt_size) {\
				if (checkFletcher16(buffer, pkt_size)) {

#define BRIDGE_BUFFER_CONCLUSION\
					cnt = 0u; buffer[0] = 0;\
				} else {\
					BRIDGE_pktDrops++;\
					cnt = 1u; buffer[0] = 0;\
				}\
			}\
		} else {\
			cnt = 0u; buffer[0] = 0;\
		}\
	}
#endif

/**
 * @}
 */ 

/** @defgroup BRIDGE_Private_Variables
 * @{
 */ 
static uint32_t BRIDGE_pktDrops = 0u;
/**
 * @}
 */ 

/** @defgroup BRIDGE_Private_Functions
 * @{
 */ 
void BRIDGE_HandleReceiverPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandlePressurePkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleAirDataPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleMHPPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleIMUPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleAccelPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleGyroPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleMagPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleActuatorPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleGNSSPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleGNSSUTCPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleGNSSUTCWPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleGNSSLLAPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleGNSSVelPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleGNSSHealthPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleGNSSHealth2Pkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleGNSSRTCMPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleGNSSSVINPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleSupplyPkt(uint8_t *byte,uint8_t size);

void BRIDGE_HandleNDVIUpPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleNDVIDownPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleNDVIPkt(uint8_t *byte,uint8_t size);

void BRIDGE_HandleAGLPkt(uint8_t *byte,uint8_t size);
void BRIDGE_HandleProximityPkt(uint8_t *byte,uint8_t size);

void BRIDGE_HandleADSBPkt(uint8_t *byte,uint8_t size);

void updateGPSValues(
		float ts, int16_t w, uint8_t h, uint8_t m, float s,
		double latitude, double longitude, float altitude,
		float vel_n, float vel_e, float vel_d,
		float course, float sog,
		float pdop, uint8_t satellites, uint8_t fix_type);

void updateGPSUTCValues(
		float ts, uint16_t w, uint8_t h, uint8_t m, float s);

void updateGPSLLAValues(
		float ts,
		double latitude, double longitude, float altitude);

void updateGPSVelValues(
		float ts,
		float course, float sog,
		float vel_n, float vel_e, float vel_d);

void updateGPSHealthValues(
		float ts,
		float pdop, uint8_t satellites, uint8_t fix_type);

void updateMagValues(
		float ts,
		float mag_x,
		float mag_y,
		float mag_z);

void handleNDVI(float ts, uint8_t id, float red, float near_ir, float ir_ambient, float ir_object);


/**
 * @}
 */ 

//defined elsewhere
float getElapsedTime(void);

// ==============================================================================
// FUNCTIONS FOR HANDLING CAN-BUS PACKETS
// ==============================================================================
/**
 * @brief	Arbitrate data packet
 * @param	CAN bus frame ID
 * @param	data Pointer to data
 * @param	size Size of data [bytes]
 * @retval None
 */
void BRIDGE_Arbiter(uint32_t id, void *data_ptr, uint8_t size)
{
	uint8_t * data = (uint8_t *) data_ptr;

	switch(id)
	{
		case CAN_PKT_RECEIVER:   BRIDGE_HandleReceiverPkt(data,size); break;
		case CAN_PKT_PRESSURE:   BRIDGE_HandlePressurePkt(data,size); break;
		case CAN_PKT_AIR_DATA:   BRIDGE_HandleAirDataPkt(data,size); break;
		case CAN_PKT_MHP:        BRIDGE_HandleMHPPkt(data,size); break;
		case CAN_PKT_IMU:        BRIDGE_HandleIMUPkt(data,size); break;
		case CAN_PKT_ACCEL:      BRIDGE_HandleAccelPkt(data,size); break;
		case CAN_PKT_GYRO:       BRIDGE_HandleGyroPkt(data,size); break;
		case CAN_PKT_MAG:        BRIDGE_HandleMagPkt(data,size); break;
		case CAN_PKT_ACTUATOR:   BRIDGE_HandleActuatorPkt(data,size); break;
		case CAN_PKT_GNSS:       BRIDGE_HandleGNSSPkt(data,size); break;
		case CAN_PKT_GNSS_UTC:   BRIDGE_HandleGNSSUTCPkt(data,size); break;
		case CAN_PKT_GNSS_UTC_W: BRIDGE_HandleGNSSUTCWPkt(data,size); break;
		case CAN_PKT_GNSS_LLA:   BRIDGE_HandleGNSSLLAPkt(data,size); break;
		case CAN_PKT_GNSS_VEL:   BRIDGE_HandleGNSSVelPkt(data,size); break;
		case CAN_PKT_GNSS_HEALTH:BRIDGE_HandleGNSSHealthPkt(data,size); break;
		case CAN_PKT_GNSS_HEALTH_2:BRIDGE_HandleGNSSHealth2Pkt(data,size); break;
		case CAN_PKT_GNSS_RTCM:  BRIDGE_HandleGNSSRTCMPkt(data,size); break;
		case CAN_PKT_GNSS_SVIN:  BRIDGE_HandleGNSSSVINPkt(data,size); break;
		case CAN_PKT_SUPPLY:     BRIDGE_HandleSupplyPkt(data,size); break;
		case CAN_PKT_NDVI:       BRIDGE_HandleNDVIPkt(data,size); break;
		case CAN_PKT_NDVI_UP:    BRIDGE_HandleNDVIUpPkt(data,size); break;
		case CAN_PKT_NDVI_DOWN:  BRIDGE_HandleNDVIDownPkt(data,size); break;
		case CAN_PKT_AGL:	       BRIDGE_HandleAGLPkt(data,size); break;
		case CAN_PKT_PROXIMITY:	 BRIDGE_HandleProximityPkt(data,size); break;
		case CAN_PKT_ADSB:    	 BRIDGE_HandleADSBPkt(data,size); break;
		default: break;
	}
}


/**
 * @brief	Handle receiver packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleReceiverPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core
	static uint8_t pkt_size = sizeof(CAN_Receiver_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleReceiverPkt";
#endif
	static uint8_t buffer[sizeof(CAN_Receiver_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//
	if(!can_pwm_in) can_pwm_in = 1u;

	CAN_Receiver_t *data = (CAN_Receiver_t *)buffer;

	float t0 = getElapsedTime();
	updatePWMIn(t0, data->usec);
#ifndef ARCH_stm32f1
	pwm_in_count++;
#endif

	pmesg(VERBOSE_CAN, "RECEIVER: %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d\r\n",
			data->usec[0],data->usec[1],data->usec[2],data->usec[3],
			data->usec[4],data->usec[5],data->usec[6],data->usec[7],
			data->usec[8],data->usec[9],data->usec[10],data->usec[11],
			data->usec[12],data->usec[13],data->usec[14],data->usec[15]);
	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief	Handle pressure packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandlePressurePkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core
	static uint8_t pkt_size = sizeof(CAN_Pressure_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandlePressurePkt";
#endif
	static uint8_t buffer[sizeof(CAN_Pressure_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//
	
	if(!can_pressure_sensor) can_pressure_sensor = 1u;

	CAN_Pressure_t *data;
	data = (CAN_Pressure_t *)buffer;

	float t0 = getElapsedTime();
	if(data->pressureSta > -FLT_MAX) {
		//FIXME -- sensors is expecting hPa, should be expecting Pa
		updateStaticPressure(t0, data->pressureSta, data->temp);
	}
	if(data->pressureDyn > -FLT_MAX) {
		//FIXME -- sensors is expecting hPa, should be expecting Pa
		//FIXME -- changed back to expecting Pa, I guess this is correct, but needs to be checked
		updateDynamicPressure(t0, data->pressureDyn, data->temp);
	}
	if(data->temp > -FLT_MAX) {
		updateTemperature(t0, data->temp);
	}
#ifndef ARCH_stm32f1
	pressure_count++;
#endif

	// DEBUG
	pmesg(VERBOSE_CAN, "PRESSURE: %+.5f [Pa], %+.5f [Pa], %+.2f [deg C]\n\r", 
			data->pressureSta, data->pressureDyn, data->temp);

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief	Handle air data packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleAirDataPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core
	static uint8_t pkt_size = sizeof(CAN_AirData_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleAirDataPkt";
#endif
	static uint8_t buffer[sizeof(CAN_AirData_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//
	
	if(!can_pressure_sensor) can_pressure_sensor = 1u;

	CAN_AirData_t *data;
	data = (CAN_AirData_t *)buffer;

	float t0 = getElapsedTime();
	if(data->static_pressure > -FLT_MAX) {
		updateStaticPressure(t0, data->static_pressure, data->air_temperature);
	}
	if(data->dynamic_pressure > -FLT_MAX) {
		updateDynamicPressure(t0, data->dynamic_pressure, data->air_temperature);
	}
	if(data->air_temperature > -FLT_MAX) {
		updateTemperature(t0, data->air_temperature);
	}
	if(data->humidity > -FLT_MAX) {
		updateHumidity(t0, data->humidity);
	}

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief	Handle MHP packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleMHPPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core
	static uint8_t pkt_size = sizeof(CAN_MHP_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleMHPPkt";
#endif
	static uint8_t buffer[sizeof(CAN_MHP_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//
	
	if(!can_pressure_sensor) can_pressure_sensor = 1u;

	CAN_MHP_t *data;
	data = (CAN_MHP_t *)buffer;

	float t0 = getElapsedTime();
#if 0
	updateMHP(t0, 
			data->static_pressure,
			data->dynamic_pressure,
			data->air_temperature,
			data->humidity,
			data->gyroscope,
			data->accelerometer,
			data->magnetometer,
			data->alpha,
			data->beta);
#else
	uint8_t i;
	float dyn_p[5];
	float gyr[3];
	float acc[3];
	float mag[3];

	for(i=0;i<5;i++) 
		dyn_p[i] = (float)(data->dynamic_pressure[i])/1000.0;
	for(i=0;i<3;i++) {
			gyr[i] = (float)(data->gyroscope[i])/1000.0;
			acc[i] = (float)(data->accelerometer[i])/1000.0;
			mag[i] = (float)(data->magnetometer[i])/100.0;
	}

	updateMHP(
			t0, 
			//(float)(data->system_time)/1000.0,
			(float)(data->static_pressure)/1000.0,
			dyn_p,
			(float)(data->air_temperature)/100.0,
			(float)(data->humidity)/100.0,
			gyr,
			acc,
			mag,
			(float)(data->alpha)/1000.0,
			(float)(data->beta)/1000.0);
#endif

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief	Handle IMU packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleIMUPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core
	static uint8_t pkt_size = sizeof(CAN_IMU_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleIMUPkt";
#endif
	static uint8_t buffer[sizeof(CAN_IMU_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//
	
	if(!can_imu_sensor) can_imu_sensor = 1u;

	CAN_IMU_t *data;
	data = (CAN_IMU_t *)buffer;

#ifndef _TEST
	float t0 = getElapsedTime();

	updateIMU(t0, 
			data->ax, data->ay, data->az, 
			data->gx, data->gy, data->gz, 
			data->mx, data->my, data->mz);

#ifndef ARCH_stm32f1
	imu_count++;
#endif
#endif

	pmesg(VERBOSE_CAN, "IMU: %+.3f, %+.3f, %+.3f [g], %+.3f, %+.3f, %+.3f [rad/s], %+.2f [deg C]\n\r", 
			data->ax, data->ay, data->az, data->gx, data->gy, data->gz, data->temp);

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief	Handle Accelerometer packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleAccelPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core
	static uint8_t pkt_size = sizeof(CAN_Accelerometer_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleAccelPkt";
#endif
	static uint8_t buffer[sizeof(CAN_Accelerometer_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//

	if(!can_imu_sensor) can_imu_sensor = 1u;

	CAN_Accelerometer_t *data;
	data = (CAN_Accelerometer_t *)buffer;

#ifndef _TEST
	float t0 = getElapsedTime();

	updateAccelerometer(t0, 
			data->ax, data->ay, data->az);

	//accel_count++;
#endif

	pmesg(VERBOSE_CAN, "ACC: %+.3f, %+.3f, %+.3f [g]\n",
			data->ax, data->ay, data->az, data->temp);

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief	Handle Gyroscope packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleGyroPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core
	static uint8_t pkt_size = sizeof(CAN_Gyroscope_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleGyroPkt";
#endif
	static uint8_t buffer[sizeof(CAN_Gyroscope_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//
		
	if(!can_imu_sensor) can_imu_sensor = 1u;

	CAN_Gyroscope_t *data;
	data = (CAN_Gyroscope_t *)buffer;

#ifndef _TEST
	float t0 = getElapsedTime();

	updateGyroscope(t0, 
			data->gx, data->gy, data->gz);

	//gyro_count++;
#endif

	pmesg(VERBOSE_CAN, "GYR: %+.3f, %+.3f, %+.3f [rad/s]\n",
			data->gx, data->gy, data->gz, data->temp);

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief	Handle Magnetometer packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleMagPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core || defined BOARD_MHP
	static uint8_t pkt_size = sizeof(CAN_Magnetometer_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleMagPkt";
#endif
	static uint8_t buffer[sizeof(CAN_Magnetometer_t)];

	BRIDGE_BUFFER_PREAMBLE

		//----- packet specific code -----//

		if(!can_mag_sensor) can_mag_sensor = 1u;

	CAN_Magnetometer_t *data;
	data = (CAN_Magnetometer_t *)buffer;

	float t0 = getElapsedTime();

#if defined IMPLEMENTATION_firmware // FIXME - make the same between firmware and hardware
	updateMagValues(t0, 
			data->mx, data->my, data->mz);
	//mag_count++;
#else
	updateMagnetometer(t0, 
			data->mx, data->my, data->mz);
	//mag_count++;
#endif

	pmesg(VERBOSE_CAN, "MAG: %+.3f, %+.3f, %+.3f [uT]\n",
			data->mx, data->my, data->mz);

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief	Handle actuator packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
#if defined _SP_RECEIVER || defined _SP_FUTABA
extern uint32_t last_actuator_command;
#endif
#if defined _SP_ACTUATOR_HACKHD
static camera_triggered = 0u;
#endif
void BRIDGE_HandleActuatorPkt(uint8_t *byte, uint8_t size)
{
#if defined _SP_ACTUATOR || defined IMPLEMENTATION_xplane || defined _SP_RECEIVER || defined _SP_FUTABA || defined _SP_ACTUATOR_HACKHD || defined _SP_ACTUATOR_A6000 || defined SDK
	static uint8_t pkt_size = sizeof(CAN_Actuator_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleActuatorPkt";
#endif
	static uint8_t buffer[sizeof(CAN_Actuator_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//
	
	CAN_Actuator_t *data;
	data = (CAN_Actuator_t *)buffer;

#if defined _SP_ACTUATOR || defined _SP_ACTUATOR_HACKHD || defined _SP_ACTUATOR_A6000
#ifdef _SP_ACTUATOR
	LED_Toggle(0);	
#if 1
	PWM_SetPulseWidth(0, data->usec[DIP_GetVal()]);
#else
	uint16_t usec = data->usec[DIP_GetVal()];

	if(usec > 800 && usec < 1400)
		PWM_SetPulseWidth(0,1000);
	if(usec > 1600 && usec < 2300)
		PWM_SetPulseWidth(0,2000);
#endif
#endif
#ifdef _SP_ACTUATOR_HACKHD
	if(data->usec[DIP_GetVal()] > 1500 && !camera_triggered) {
		camera_triggered = 1u;
		LED_On(0);	
		LED_On(1);
		Delay(500);
		LED_Off(1);
		Delay(1500); // wait for picture
		LED_Off(0);
	}

	if(data->usec[DIP_GetVal()] < 1500 && camera_triggered) {
		camera_triggered = 0u;
	}
#endif
#ifdef _SP_ACTUATOR_A6000
	updateActuator(data->usec[DIP_GetVal()]);
#endif
#else
	uint16_t actuator_values[CAN_NUM_ACTUATORS];
	uint8_t i;

	for(i=0; i<CAN_NUM_ACTUATORS; i++) {
		actuator_values[i] = data->usec[i];
	}

#if !defined _SP_RECEIVER && !defined _SP_FUTABA
	updateActuatorValues(actuator_values);
#endif
#ifdef IMPLEMENTATION_xplane
	/*
	static float last_time = 0;
	float now = getElapsedTime();
	float dT = now - last_time;
	last_time = now;
	pmesg(VERBOSE_CAN, "ACTUATOR: dT=%.5f\n", dT);
	*/
#endif


#endif

	pmesg(VERBOSE_CAN, "ACTUATOR: %04d %04d %04d %04d %04d %04d %04d %04d %04d %04d\r\n",
			data->usec[0],data->usec[1],data->usec[2],data->usec[3],
			data->usec[4],data->usec[5],data->usec[6],data->usec[7],
			data->usec[8],data->usec[9]);

#ifdef VERBOSE
#ifdef ARCH_stm32f4
	LED_Blink(0,100);
#endif
#endif

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION

//#endif
#if defined _SP_RECEIVER || defined _SP_FUTABA
	last_actuator_command = GetTimeU();
#endif
#endif
}


/**
 * @brief	Handle GNSS packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleGNSSPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core
	static uint8_t pkt_size = sizeof(CAN_GNSS_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleGNSSPkt";
#endif
	static uint8_t buffer[sizeof(CAN_GNSS_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//

	if(!can_gps_sensor) can_gps_sensor = 1u;

	CAN_GNSS_t *data;
	data = (CAN_GNSS_t *)buffer;

	float t0 = getElapsedTime();
#if defined IMPLEMENTATION_firmware // FIXME - make the same between firmware and hardware
	updateGPSValues(t0, data->week, data->hours, data->minutes, data->seconds,
			data->latitude, data->longitude, data->altitude,
			data->vx, data->vy, data->vz,
			data->heading, data->speed,
			data->pdop, data->satellites, data->fix_type);
#ifndef ARCH_stm32f1
	gps_count++;
#endif
#else
	updateGPS(t0, data->week, data->hours, data->minutes, data->seconds,
			data->latitude, data->longitude, data->altitude,
			data->vx, data->vy, data->vz,
			data->heading, data->speed,
			data->pdop, data->satellites, data->fix_type);
#ifndef ARCH_stm32f1
	gps_count++;
#endif
#endif

	pmesg(VERBOSE_CAN, "GNSS: %02d:%02d:%02.1f | %+.1f %+.1f %+.1f | %+.1f %+.1f | %+.1f %d\n\r", 
			data->hours, data->minutes, data->seconds, 
			data->latitude, data->longitude, data->altitude, 
			data->heading, data->speed, data->pdop, data->satellites);

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief	Handle GNSS packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleGNSSUTCPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core || defined BOARD_MHP
	static uint8_t pkt_size = sizeof(CAN_GNSS_UTC_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleGNSSUTCPkt";
#endif
	static uint8_t buffer[sizeof(CAN_GNSS_UTC_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//

	if(!can_gps_sensor) can_gps_sensor = 1u;

	CAN_GNSS_UTC_t *data;
	data = (CAN_GNSS_UTC_t *)buffer;

	float t0 = getElapsedTime();
	
	updateGPSUTCValues(t0, 0, data->hours, data->minutes, data->seconds);

	pmesg(VERBOSE_CAN, "GNSS: %02d:%02d:%02.1f\n\r",
			data->hours, data->minutes, data->seconds);

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief	Handle GNSS packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleGNSSUTCWPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core || defined BOARD_MHP
	static uint8_t pkt_size = sizeof(CAN_GNSS_UTC_W_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleGNSSUTCWPkt";
#endif
	static uint8_t buffer[sizeof(CAN_GNSS_UTC_W_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//

	if(!can_gps_sensor) can_gps_sensor = 1u;

	CAN_GNSS_UTC_W_t *data;
	data = (CAN_GNSS_UTC_W_t *)buffer;

	float t0 = getElapsedTime();
	
	updateGPSUTCValues(t0, data->week, data->hours, data->minutes, data->seconds);

	pmesg(VERBOSE_CAN, "GNSS: %02d:%02d:%02.1f\n\r",
			data->hours, data->minutes, data->seconds);

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief	Handle GNSS packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleGNSSLLAPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core || defined BOARD_MHP
  static uint8_t pkt_size = sizeof(CAN_GNSS_LLA_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleGNSSLLAPkt";
#endif
	static uint8_t buffer[sizeof(CAN_GNSS_LLA_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//

	if(!can_gps_sensor) can_gps_sensor = 1u;

	CAN_GNSS_LLA_t *data;
	data = (CAN_GNSS_LLA_t *)buffer;

	float t0 = getElapsedTime();

	updateGPSLLAValues(t0,
			data->latitude, data->longitude, data->altitude);
#ifndef ARCH_stm32f1
	gps_count++;
#endif

	pmesg(VERBOSE_CAN, "GNSS: %+.1f %+.1f %+.1f\n\r", 
			data->latitude, data->longitude, data->altitude);

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief	Handle GNSS packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleGNSSVelPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core || defined BOARD_MHP
  static uint8_t pkt_size = sizeof(CAN_GNSS_VEL_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleGNSSVelPkt";
#endif
	static uint8_t buffer[sizeof(CAN_GNSS_VEL_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//

	if(!can_gps_sensor) can_gps_sensor = 1u;

	CAN_GNSS_VEL_t *data;
	data = (CAN_GNSS_VEL_t *)buffer;

	float t0 = getElapsedTime();
	
	updateGPSVelValues(t0,
			data->heading, data->speed,
			data->vx, data->vy, data->vz);

	pmesg(VERBOSE_CAN, "GNSS: %+.1f %+.1f %+.1f | %+.1f %+.1f\n\r", 
			data->vx, data->vy, data->vz,
			data->heading, data->speed);

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief	Handle GNSS packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleGNSSHealth2Pkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core || defined BOARD_MHP
  static uint8_t pkt_size = sizeof(CAN_GNSS_HEALTH_2_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleGNSSHealth2Pkt";
#endif

	static uint8_t buffer[sizeof(CAN_GNSS_HEALTH_2_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//

	if(!can_gps_sensor) can_gps_sensor = 1u;

	CAN_GNSS_HEALTH_2_t *data;
	data = (CAN_GNSS_HEALTH_2_t *)buffer;

	float t0 = getElapsedTime();
	
	updateGPSHealthValues(t0, data->pdop, data->satellites, data->fix_type);

	pmesg(VERBOSE_CAN, "GNSS: %+.1f %d\n\r", data->pdop, data->satellites);

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief	Handle GNSS packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleGNSSHealthPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core || defined BOARD_MHP
  static uint8_t pkt_size = sizeof(CAN_GNSS_HEALTH_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleGNSSHealthPkt";
#endif
	static uint8_t buffer[sizeof(CAN_GNSS_HEALTH_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//

	if(!can_gps_sensor) can_gps_sensor = 1u;

	CAN_GNSS_HEALTH_t *data;
	data = (CAN_GNSS_HEALTH_t *)buffer;

	float t0 = getElapsedTime();
	
	updateGPSHealthValues(t0, data->pdop, data->satellites, 0);

	pmesg(VERBOSE_CAN, "GNSS: %+.1f %d\n\r", data->pdop, data->satellites);

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief	Handle GNSS packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleGNSSRTCMPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_GNSS || defined BOARD_core
	static uint8_t pkt_size = sizeof(CAN_GNSS_RTCM_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleGNSSRTCMPkt";
#endif
	static uint8_t buffer[sizeof(CAN_GNSS_RTCM_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//

	if(!can_gps_sensor) can_gps_sensor = 1u;

	CAN_GNSS_RTCM_t *data;
	data = (CAN_GNSS_RTCM_t *)buffer;

	float t0 = getElapsedTime();
	
	updateGPSRTCM(t0, data->size, data->payload);

	pmesg(VERBOSE_CAN, "GNSS RTCM Data\n\r");

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief	Handle GNSS packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleGNSSSVINPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_GCS
	static uint8_t pkt_size = sizeof(CAN_GNSS_SVIN_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleGNSSSVINPkt";
#endif
	static uint8_t buffer[sizeof(CAN_GNSS_SVIN_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//

	if(!can_gps_sensor) can_gps_sensor = 1u;

	CAN_GNSS_SVIN_t *data;
	data = (CAN_GNSS_SVIN_t *)buffer;
	
	updateGPSSVIN( 
			data->time_elapsed,
			data->time_minimum,
			data->accuracy,
			data->accuracy_minimum,
			data->flags);

	pmesg(VERBOSE_CAN, "GNSS SVIN Data\n\r");

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief	Handle supply packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleSupplyPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core
	static uint8_t pkt_size = sizeof(CAN_Supply_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleSupplyPkt";
#endif
	static uint8_t buffer[sizeof(CAN_Supply_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//
	
	CAN_Supply_t *data;
	data = (CAN_Supply_t *)buffer;

#ifndef _TEST
	float t0 = getElapsedTime();
	updateSupply(t0,data->voltage,data->current, data->coulomb_count,data->temperature);
#endif

	pmesg(VERBOSE_CAN, "SUPPLY: %.01f V, %.01f A, %.01f mAh, %.01f deg C\n\r", data->voltage, data->current, data->coulomb_count, data->temperature);

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief   Handle NDVI packet
 * @param   byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleNDVIUpPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_arbiter || defined BOARD_core
	static uint8_t pkt_size = sizeof(CAN_NDVI_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleNDVIUpPkt";
#endif
	static uint8_t buffer[sizeof(CAN_NDVI_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//
	
	CAN_NDVI_t *data;
	data = (CAN_NDVI_t *)buffer;
	handleNDVI(getElapsedTime(), data->id, data->red, data->near_ir, data->ir_ambient, data->ir_object);

	// DEBUG - sanity check
	pmesg(VERBOSE_CAN, "NDVI %02u: %0.02f ?, %0.02f ?, %0.02f deg C, %0.02f deg C\n\r",
			data->id, data->red, data->near_ir, data->ir_ambient, data->ir_object);

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief   Handle NDVI packet
 * @param   byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleNDVIDownPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_arbiter || defined BOARD_core
	static uint8_t pkt_size = sizeof(CAN_NDVI_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleNDVIDownPkt";
#endif
	static uint8_t buffer[sizeof(CAN_NDVI_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//

	CAN_NDVI_t *data;
	data = (CAN_NDVI_t *)buffer;
	handleNDVI(getElapsedTime(), data->id, data->red, data->near_ir, data->ir_ambient, data->ir_object);

	// DEBUG - sanity check
	pmesg(VERBOSE_CAN, "NDVI %02u: %0.02f ?, %0.02f ?, %0.02f deg C, %0.02f deg C\n\r",
			data->id, data->red, data->near_ir, data->ir_ambient, data->ir_object);

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}


/**
 * @brief	Handle NDVI packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleNDVIPkt(uint8_t *byte, uint8_t size)
{
#ifdef BOARD_pro_arbiter
	static uint8_t pkt_size = sizeof(CAN_NDVI_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleNDVIPkt";
#endif
	static uint8_t buffer[sizeof(CAN_NDVI_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//
	
	CAN_NDVI_t *data;
	data = (CAN_NDVI_t *)buffer;
	handleNDVI(getElapsedTime(), data->id, data->red, data->near_ir, data->ir_ambient, data->ir_object);

	// DEBUG - sanity check
	pmesg(VERBOSE_CAN, "NDVI %02u: %0.02f ?, %0.02f ?, %0.02f deg C, %0.02f deg C\n\r", 
			data->id, data->red, data->near_ir, data->ir_ambient, data->ir_object);

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}


/**
 * @brief	Handle AGL packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleAGLPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core
	static uint8_t pkt_size = sizeof(CAN_AGL_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleAGLPkt";
#endif
	static uint8_t buffer[sizeof(CAN_AGL_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//
	
	CAN_AGL_t *data;
	data = (CAN_AGL_t *)buffer;
	//updateAGL(getElapsedTime(), data->distance, data->velocity);
	updateAGL(getElapsedTime(), data->distance);

	// DEBUG - sanity check
	pmesg(VERBOSE_CAN, "AGL: %0.02f s, %0.02f m, %0.02f m/s\n", 
			data->timestamp, data->distance, data->velocity);

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

/**
 * @brief	Handle proximity packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleProximityPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core && defined IMPLEMENTATION_firmware
	static uint8_t pkt_size = sizeof(CAN_Proximity_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleProximityPkt";
#endif
	static uint8_t buffer[sizeof(CAN_Proximity_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//
	
#ifdef VERBOSE
	CAN_Proximity_t *data = (CAN_Proximity_t *)buffer;
#endif
	//updateProximity(getElapsedTime(), data->distance, data->velocity);
	//updateProximity(getElapsedTime(), data->x, data->y, data->z, data->distance);

	// DEBUG - sanity check
	pmesg(VERBOSE_CAN, "PROXIMITY: %0.02f s, %0.02f m, %0.02f m/s\n", 
			data->timestamp, data->distance, data->velocity);

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}


/**
 * @brief	Handle ADSB packet
 * @param	byte Pointer to the byte of data
 * @retval None
 */
void BRIDGE_HandleADSBPkt(uint8_t *byte, uint8_t size)
{
#if defined BOARD_core
	static uint8_t pkt_size = sizeof(CAN_ADSB_t);
#ifdef DEBUG
	static char * function_name = "BRIDGE_HandleADSBPkt";
#endif
	static uint8_t buffer[sizeof(CAN_ADSB_t)];

	BRIDGE_BUFFER_PREAMBLE

	//----- packet specific code -----//

	CAN_ADSB_t *data = (CAN_ADSB_t *)buffer;

	updateADSB(data->timestamp,
			data->icao_address,
			data->latitude,
			data->longitude,
			data->altitude_type,
			data->altitude,
			data->heading,
			data->horizontal_velocity,
			data->vertical_velocity,
			data->callsign,
			data->emitter_type,
			data->tslc,
			data->flags,
			data->squawk);

#ifdef VERBOSE
	// DEBUG - sanity check
	pmesg(VERBOSE_CAN, "ADSB: %0.02f s, %s: %0.01f deg, %0.01f deg, %0.01f m\n", 
			data->timestamp, data->callsign, data->latitude, data->longitude, data->altitude);
#endif

	//----- packet specific code -----//

	BRIDGE_BUFFER_CONCLUSION
#endif
}

// ==============================================================================
// FUNCTIONS FOR SENDING CAN-BUS PACKETS
// ==============================================================================

/**
 * @brief	Send receiver packet
 * @param	p CAN peripheral ID
 * @param	usecx Pointer to receiver pulse widths [usec]
 * @retval None
 */
uint8_t BRIDGE_SendReceiverPkt(uint8_t p, uint8_t num_channels, uint16_t *usec)
{
	CAN_Receiver_t data;
	uint8_t i=0;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	for(i=0; i< num_channels; i++)
		data.usec[i] = usec[i]; // [usec]
	for(i=num_channels; i< 16; i++)
		data.usec[i] = 0; // [usec]
	setFletcher16((uint8_t *)(&data), sizeof(CAN_Receiver_t));

	//uint8_t tries = 0;
	//while(!CAN_Write(p, CAN_PKT_RECEIVER, &data, sizeof(CAN_Receiver_t)) && tries++ < 20) Delay(100);
	return CAN_Write(p, CAN_PKT_RECEIVER, &data, sizeof(CAN_Receiver_t)) == sizeof(CAN_Receiver_t);
}

/**
 * @brief	Send pressure packet
 * @param	p CAN peripheral ID
 * @param	pressureSta Static pressure [hPa]
 * @param	pressureDyn Dynamic pressure [hPa]
 * @param	temp Temperature [deg C]
 * @retval None
 */
uint8_t BRIDGE_SendPressurePkt(uint8_t p, float pressureSta, float pressureDyn, float temp)
{
	CAN_Pressure_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	data.pressureSta = pressureSta;
	data.pressureDyn = pressureDyn;
	data.temp = temp;
	setFletcher16((uint8_t *)(&data), sizeof(CAN_Pressure_t));

	return CAN_Write(p, CAN_PKT_PRESSURE, &data, sizeof(CAN_Pressure_t));
}

/**
 * @brief	Send air data packet
 * @param	p CAN peripheral ID
 * @retval None
 */
uint8_t BRIDGE_SendAirDataPkt(uint8_t p,
		float static_pressure,  // [Pa]
		float dynamic_pressure,  // [Pa]
		float air_temperature,  // [deg C]
		float humidity  // [%]
		) {

	CAN_AirData_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;

	data.static_pressure = static_pressure;
	data.dynamic_pressure = dynamic_pressure;
	data.air_temperature = air_temperature;
	data.humidity = humidity;

	setFletcher16((uint8_t *)(&data), sizeof(CAN_AirData_t));

	return CAN_Write(p, CAN_PKT_AIR_DATA, &data, sizeof(CAN_AirData_t));
}

/**
 * @brief	Send MHP packet
 * @param	p CAN peripheral ID
 * @retval None
 */
uint8_t BRIDGE_SendMHPPkt(uint8_t p,
		uint8_t error_code,
		float system_time,  // [s]
		float static_pressure,  // [Pa]
		float dynamic_pressure[5],  // [Pa]
		float air_temperature,  // [deg C]
		float humidity,  // [%]
		float gyroscope[3], // [rad/s]
		float accelerometer[3], //[g]
		float magnetometer[3], //[uT]
		float alpha,  // [rad]
		float beta,  // [rad]
		float q,  // [m/s]
		float ias,  // [m/s]
		float tas  // [m/s]
		) {

	uint8_t i=0;
	static CAN_MHP_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;

#if 0
	//data.error_code = error_code;
	data.system_time = system_time;
	data.static_pressure = static_pressure;
	for(i=0; i<5; i++) data.dynamic_pressure[i] = dynamic_pressure[i];
	data.air_temperature = air_temperature;
	data.humidity = humidity;
	for(i=0; i<3; i++) data.gyroscope[i] = gyroscope[i];
	for(i=0; i<3; i++) data.accelerometer[i] = accelerometer[i];
	for(i=0; i<3; i++) data.magnetometer[i] = magnetometer[i];
	data.alpha = alpha;
	data.beta = beta;
	//data.q = q;
	//data.ias = ias;
	//data.tas = tas;
#else
	data.system_time = (uint32_t)(system_time*1000);
	data.static_pressure = (uint32_t)(static_pressure*1000);
	for(i=0; i<5; i++) data.dynamic_pressure[i] = (int32_t)(dynamic_pressure[i]*1000.f);
	data.air_temperature = (int16_t)(air_temperature*100);
	data.humidity = (uint16_t)(humidity*100);
	for(i=0; i<3; i++) data.gyroscope[i] = (int16_t)(gyroscope[i]*1000);
	for(i=0; i<3; i++) data.accelerometer[i] = (int16_t)(accelerometer[i]*1000);
	for(i=0; i<3; i++) data.magnetometer[i] = (int16_t)(magnetometer[i]*100);
	data.alpha = (int16_t)(alpha*1000);
	data.beta = (int16_t)(beta*1000);
#endif

	setFletcher16((uint8_t *)(&data), sizeof(CAN_MHP_t));

	return CAN_Write(p, CAN_PKT_MHP, &data, sizeof(CAN_MHP_t));
}

/**
 * @brief	Send IMU packet
 * @param	p CAN peripheral ID
 * @param	ax Accelerometer value along x-axis [g]
 * @param	ay Accelerometer value along y-axis [g]
 * @param	az Accelerometer value along z-axis [g]
 * @param	gx Rate-gyroscope value along x-axis [rad/s]
 * @param	gy Rate-gyroscope value along y-axis [rad/s]
 * @param	gz Rate-gyroscope value along z-axis [rad/s]
 * @param	temp Temperature [deg C]
 * @retval None
 */
uint8_t BRIDGE_SendIMUPkt(uint8_t p, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float temp)
{
	CAN_IMU_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	data.ax = ax;
	data.ay = ay;
	data.az = az;
	data.gx = gx;
	data.gy = gy;
	data.gz = gz;
	data.mx = mx;
	data.my = my;
	data.mz = mz;
	data.temp = temp;
	setFletcher16((uint8_t *)(&data), sizeof(CAN_IMU_t));

	return CAN_Write(p, CAN_PKT_IMU, &data, sizeof(CAN_IMU_t));
}

/**
 * @brief	Send Accelerometer packet
 * @param	p CAN peripheral ID
 * @param	ax Accelerometer value along x-axis [g]
 * @param	ay Accelerometer value along y-axis [g]
 * @param	az Accelerometer value along z-axis [g]
 * @param	temp Temperature [deg C]
 * @retval None
 */
uint8_t BRIDGE_SendAccelPkt(uint8_t p, float ax, float ay, float az, float temp)
{
	CAN_Accelerometer_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	data.ax = ax;
	data.ay = ay;
	data.az = az;
	data.temp = temp;
	setFletcher16((uint8_t *)(&data), sizeof(CAN_Accelerometer_t));

	return CAN_Write(p, CAN_PKT_ACCEL, &data, sizeof(CAN_Accelerometer_t));
}

/**
 * @brief	Send Gyroscope packet
 * @param	p CAN peripheral ID
 * @param	gx Rate-gyroscope value along x-axis [rad/s]
 * @param	gy Rate-gyroscope value along y-axis [rad/s]
 * @param	gz Rate-gyroscope value along z-axis [rad/s]
 * @param	temp Temperature [deg C]
 * @retval None
 */
uint8_t BRIDGE_SendGyroPkt(uint8_t p, float gx, float gy, float gz, float temp)
{
	CAN_Gyroscope_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	data.gx = gx;
	data.gy = gy;
	data.gz = gz;
	data.temp = temp;
	setFletcher16((uint8_t *)(&data), sizeof(CAN_Gyroscope_t));

	return CAN_Write(p, CAN_PKT_GYRO, &data, sizeof(CAN_Gyroscope_t));
}

/**
 * @brief	Send Magnetometer packet
 * @param	p CAN peripheral ID
 * @param	mx Magnetic field strength value along x-axis [uT]
 * @param	my Magnetic field strength value along y-axis [uT]
 * @param	mz Magnetic field strength value along z-axis [uT]
 * @retval None
 */
uint8_t BRIDGE_SendMagPkt(uint8_t p, float mx, float my, float mz)
{
	CAN_Magnetometer_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	data.mx = mx;
	data.my = my;
	data.mz = mz;
	setFletcher16((uint8_t *)(&data), sizeof(CAN_Magnetometer_t));

	return CAN_Write(p, CAN_PKT_MAG, &data, sizeof(CAN_Magnetometer_t));
}

/**
 * @brief	Send actuator packet
 * @param	p CAN peripheral ID
 * @param	usec Pointer to actuator pulse width [usec]; array length is limited by CAN_NUM_ACTUATORS
 * @retval None
 */
uint8_t BRIDGE_SendActuatorPkt(uint8_t p, uint16_t *usec)
{
	uint8_t i;
	CAN_Actuator_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	for (i=0; i<CAN_NUM_ACTUATORS; i++)
		data.usec[i] = usec[i]; // [usec]
	
	setFletcher16((uint8_t *)(&data), sizeof(CAN_Actuator_t));

	//uint8_t tries = 0;
	//while(!CAN_Write(p, CAN_PKT_ACTUATOR, &data, sizeof(CAN_Actuator_t)) && tries++ < 20) Delay(100);
	return CAN_Write(p, CAN_PKT_ACTUATOR, &data, sizeof(CAN_Actuator_t));
#if defined ARCH_stm32f4
	if(UART_HWIL > 0)
		simulatedCANWrite(p, CAN_PKT_ACTUATOR, &data, sizeof(CAN_Actuator_t));
#endif
}

/**
 * @brief	Send GNSS packet
 * @param	p CAN peripheral ID
 * @param	hours Hour value [hrs]
 * @param	minutes Minute value [min]
 * @param	seconds Seconds value [sec]
 * @param	latitude Latitude value [deg]
 * @param	longitude Longitude value [deg]
 * @param	altitude Altitude value [m]
 * @param	heading Heading/Course value [deg]
 * @param	speed Speed value [m/s]
 * @param	pdop [-] 
 * @param	satellites Number of visible satellites [-]
 * @param	x-speed Speed value [m/s] N
 * @param	y-speed Speed value [m/s] E
 * @param	z-speed Speed value [m/s] D
 * @retval None
 */
uint8_t BRIDGE_SendGNSSPkt(uint8_t p,
		uint16_t week,
		uint8_t hours, uint8_t minutes, float seconds, double latitude,
		double longitude, float altitude, float heading, float speed, float pdop, uint8_t satellites, uint8_t fix_type,
		float vx, float vy, float vz)
{
	CAN_GNSS_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	data.week = week;
	data.hours = hours; // [hrs]
	data.minutes = minutes; // [min]
	data.seconds = seconds; // [sec]
	data.latitude = latitude; // [deg]
	data.longitude = longitude; // [deg]
	data.altitude = altitude; // [m]
	data.heading = heading; // [deg]
	data.speed = speed; // [m/s]
	data.pdop = pdop; // [-] 
	data.satellites = satellites; // [-]
	data.fix_type = fix_type; // [-]
	data.vx = vx; // [m/s]
	data.vy = vy; // [m/s]
	data.vz = vz; // [m/s]

	setFletcher16((uint8_t *)(&data), sizeof(CAN_GNSS_t));

	return CAN_Write(p, CAN_PKT_GNSS, &data, sizeof(CAN_GNSS_t));
}

uint8_t BRIDGE_SendGNSSUTCWPkt(uint8_t p, 
		uint16_t week,
		uint8_t hours, uint8_t minutes, float seconds) {
	CAN_GNSS_UTC_W_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	data.week = week;
	data.hours = hours; // [hrs]
	data.minutes = minutes; // [min]
	data.seconds = seconds; // [sec]

	setFletcher16((uint8_t *)(&data), sizeof(CAN_GNSS_UTC_W_t));

	return CAN_Write(p, CAN_PKT_GNSS_UTC_W, &data, sizeof(CAN_GNSS_UTC_W_t)) == sizeof(CAN_GNSS_UTC_W_t);
}

uint8_t BRIDGE_SendGNSSLLAPkt(uint8_t p, 
		double latitude, double longitude, float altitude) {
	CAN_GNSS_LLA_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	data.latitude = latitude; // [deg]
	data.longitude = longitude; // [deg]
	data.altitude = altitude; // [m]

	setFletcher16((uint8_t *)(&data), sizeof(CAN_GNSS_LLA_t));

	return CAN_Write(p, CAN_PKT_GNSS_LLA, &data, sizeof(CAN_GNSS_LLA_t)) == sizeof(CAN_GNSS_LLA_t);
}

uint8_t BRIDGE_SendGNSSVelPkt(uint8_t p, 
		float heading, float speed,
		float vx, float vy, float vz) {

	CAN_GNSS_VEL_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	data.heading = heading; // [deg]
	data.speed = speed; // [m/s]
	data.vx = vx; // [m/s]
	data.vy = vy; // [m/s]
	data.vz = vz; // [m/s]

	setFletcher16((uint8_t *)(&data), sizeof(CAN_GNSS_VEL_t));

	return CAN_Write(p, CAN_PKT_GNSS_VEL, &data, sizeof(CAN_GNSS_VEL_t)) == sizeof(CAN_GNSS_VEL_t);
}

uint8_t BRIDGE_SendGNSSHealthPkt(uint8_t p, 
		float pdop, uint8_t satellites, uint8_t fix_type) {
	CAN_GNSS_HEALTH_2_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	data.pdop = pdop; // [-] 
	data.satellites = satellites; // [-]
	data.fix_type = fix_type;

	setFletcher16((uint8_t *)(&data), sizeof(CAN_GNSS_HEALTH_2_t));

	return CAN_Write(p, CAN_PKT_GNSS_HEALTH_2, &data, sizeof(CAN_GNSS_HEALTH_2_t)) == sizeof(CAN_GNSS_HEALTH_2_t);
}

uint8_t BRIDGE_SendGNSSRTCMPkt(uint8_t p, uint8_t size, uint8_t * payload) {
	CAN_GNSS_RTCM_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	data.size = size; // [-] 
	memcpy(data.payload, payload, size);

	setFletcher16((uint8_t *)(&data), sizeof(CAN_GNSS_RTCM_t));

	return CAN_Write(p, CAN_PKT_GNSS_RTCM, &data, sizeof(CAN_GNSS_RTCM_t)) == sizeof(CAN_GNSS_RTCM_t);
}

uint8_t BRIDGE_SendGNSSSVINPkt(uint8_t p,
		uint32_t time_elapsed,
		uint32_t time_minimum,
		float accuracy,
		float accuracy_minimum,
		uint8_t flags) {

	CAN_GNSS_SVIN_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	data.time_elapsed = time_elapsed;
	data.time_minimum = time_minimum;
	data.accuracy = accuracy;
	data.accuracy_minimum = accuracy_minimum;
	data.flags = flags;

	setFletcher16((uint8_t *)(&data), sizeof(CAN_GNSS_SVIN_t));

	return CAN_Write(p, CAN_PKT_GNSS_SVIN, &data, sizeof(CAN_GNSS_SVIN_t)) == sizeof(CAN_GNSS_SVIN_t);
}


/**
 * @brief	Send supply packet
 * @param	p CAN peripheral ID
 * @param	voltage Battery voltage [mV]
 * @param	capacity Battery capacity [% x100]
 * @retval None
 */
uint8_t BRIDGE_SendSupplyPkt(uint8_t p, float voltage, float current, float coulomb_count, float temperature)
{
	CAN_Supply_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	data.voltage = voltage; // [V]
	data.current = current; // [A]
	data.coulomb_count = coulomb_count; // [mAh]
	data.temperature = temperature; // [deg C]
	setFletcher16((uint8_t *)(&data), sizeof(CAN_Supply_t));

	return CAN_Write(p, CAN_PKT_SUPPLY, &data, sizeof(CAN_Supply_t));
}

typedef struct _CAN_Supply_Old_t {
	uint8_t startByte;

	uint16_t voltage;  // [mV]
	uint16_t current;  // [mA]

	uint16_t chk;

#ifdef __cplusplus
	_CAN_Supply_Old_t() {
		startByte = 0;
		voltage = 0;
		current = 0;
		chk = 0;
	}
#endif
} __attribute__ ((packed)) CAN_Supply_Old_t;

uint8_t BRIDGE_SendSupplyPkt_Old(uint8_t p, uint16_t voltage, uint16_t current)
{
	CAN_Supply_Old_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	data.voltage = voltage; // [mV]
	data.current = current; // [mA]
	setFletcher16((uint8_t *)(&data), sizeof(CAN_Supply_Old_t));

	return CAN_Write(p, CAN_PKT_SUPPLY, &data, sizeof(CAN_Supply_Old_t));
}

/**
 * @brief	Send NDVI packet
 * @param	p CAN peripheral ID
 * @param	red Red spectrum measurement [?]
 * @param	near_ir Near Infrared spectrum measurement [?]
 * @param	ir_ambient Ambient temperature measurement [deg C]
 * @param	ir_object Object temperature measurement [deg C]
 * @retval None
 */
uint8_t BRIDGE_SendNDVIPkt(uint8_t p, uint8_t id, float red, float near_ir, float ir_ambient, float ir_object) {
	CAN_NDVI_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	data.id = id; // [?]
	data.red = red; // [?]
	data.near_ir = near_ir; // [?]
	data.ir_ambient = ir_ambient; // [deg C]
	data.ir_object = ir_object; // [deg C]
	setFletcher16((uint8_t *)(&data), sizeof(CAN_NDVI_t));

	return CAN_Write(p, CAN_PKT_NDVI, &data, sizeof(CAN_NDVI_t)); // FIXME -- sending with same ID up and down packets will screw up reconstruction
}

/**
 * @brief	Send AGL packet
 * @param	p CAN peripheral ID
 * @param	ts Time stamp of measurement [s]
 * @param	distance Distance measurement [m]
 * @param	velocity Velocity measurement [m/s]
 * @retval None
 */
uint8_t BRIDGE_SendAGLPkt(uint8_t p, float *ts, float *distance, float *velocity) {
	CAN_AGL_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	data.timestamp = *ts; // [s]
	data.distance = *distance; // [m]
	data.velocity = *velocity; // [m/s]
	setFletcher16((uint8_t *)(&data), sizeof(CAN_AGL_t));

	return CAN_Write(p, CAN_PKT_AGL, &data, sizeof(CAN_AGL_t));
}

/**
 * @brief	Send Proximity packet
 * @param	p CAN peripheral ID
 * @param	ts Time stamp of measurement [s]
 * @param	distance Distance measurement [m]
 * @param	velocity Velocity measurement [m/s]
 * @retval None
 */
uint8_t BRIDGE_SendProximityPkt(uint8_t p, float ts, float distance, float velocity) {
	CAN_Proximity_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	data.timestamp = ts; // [s]
	data.distance = distance; // [m]
	data.velocity = velocity; // [m/s]
	setFletcher16((uint8_t *)(&data), sizeof(CAN_Proximity_t));

	return CAN_Write(p, CAN_PKT_PROXIMITY, &data, sizeof(CAN_Proximity_t));
}

uint8_t BRIDGE_SendADSBPkt(uint8_t p, float ts,
		uint32_t icao_address,
		double latitude,
		double longitude,
		uint8_t altitude_type,
		float altitude,
		float heading,
		float horizontal_velocity,
		float vertical_velocity,
		char callsign[9],
		uint8_t emitter_type,
		uint8_t tslc,
		uint16_t flags,
		uint16_t squawk) {

	CAN_ADSB_t data;

	// fill packet
	data.startByte = BRIDGE_START_BYTE;
	data.timestamp = ts; // [s]
	data.icao_address = icao_address;
	data.latitude = latitude;
	data.longitude = longitude;
	data.altitude_type = altitude_type;
	data.altitude = altitude;
	data.heading = heading;
	data.horizontal_velocity = horizontal_velocity;
	data.vertical_velocity =  vertical_velocity;
	memcpy(data.callsign,callsign,9);
	data.emitter_type = emitter_type;
	data.tslc = tslc;
	data.flags = flags;
	data.squawk = squawk;
	setFletcher16((uint8_t *)(&data), sizeof(CAN_ADSB_t));

	return CAN_Write(p, CAN_PKT_ADSB, &data, sizeof(CAN_ADSB_t));
}


/**
 * @brief	Return packet drops
 * @param	None
 * @retval Packet drops
 */
__inline uint32_t BRIDGE_GetPktDrop(void)
{
	return BRIDGE_pktDrops;
}

#if defined ARCH_stm32f1 || defined STM32F413xx || defined STM32F405xx
uint8_t checkFletcher16(uint8_t * data, uint8_t size) {
	uint16_t sum1 = 0;
	uint16_t sum2 = 0;

	uint8_t i;
	for(i = 0; i < size; i++ ) {
		sum1 = (sum1 + data[i]) % 255;
		sum2 = (sum2 + sum1) % 255;
	}

	return ((sum2 << 8) | sum1) == 0;
}

void setFletcher16 (uint8_t * data, uint8_t size){
	uint16_t sum1 = 0;
	uint16_t sum2 = 0;

	uint8_t i;
	for( i = 0; i < (size-2); i++ ) {	// excludes checksum bytes
		sum1 = (sum1 + data[i]) % 255;
		sum2 = (sum2 + sum1) % 255;
	}

	data[size-2] = 255 - (( sum1 + sum2 ) % 255);
	data[size-1] = 255 - (( sum1 + data[size-2] ) % 255);

	//uint16_t checksum1 = 255 - (( sum1 + sum2 ) % 255);
	//uint16_t checksum2 = 255 - (( sum1 + checksum1 ) % 255);

	//return ((checksum2 << 8) | checksum1);
}
#endif

/**
 * @}
 */ 

/**
 * @}
 */ 

/**
 * @}
 */ 
