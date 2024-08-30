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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BRIDGE_H
#define __BRIDGE_H

#ifndef ARCH_stm32f4 
  #include <inttypes.h>
#endif

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#ifdef STM32F10X_MD
#include "stm32f10x.h"
#elif STM32F40_41xxx
#include "stm32f4xx.h"
#endif

/** @addtogroup Source
  * @{
  */
 
/** @addtogroup Low_Level
  * @{
  */

/** @addtogroup BRIDGE
  * @{
  */ 

/** @defgroup BRIDGE_Private_Defines
* @{
*/ 
#define CAN_DEVICE_0 0
#define CAN_DEVICE_1 1
#define CAN_DEVICE_2 2
#define CAN_DEVICE_3 3
#define CAN_DEVICE_4 4
#define CAN_DEVICE_5 5
#define CAN_DEVICE_6 6
#define CAN_DEVICE_7 7
#define CAN_DEVICE_8 8
#define CAN_DEVICE_9 9
	  
/**
* @}
*/ 

/** @defgroup BRIDGE_Exported_Functions
  * @{
  */ 
void BRIDGE_Arbiter(uint32_t id, void *data, uint8_t size);
uint8_t BRIDGE_SendReceiverPkt(uint8_t p, uint8_t num_channels, uint16_t *usec);

uint8_t BRIDGE_SendPressurePkt(uint8_t p, float pressureSta, float pressureDyn, float temp);
uint8_t BRIDGE_SendAirDataPkt(uint8_t p,
		float static_pressure,  // [Pa]
		float dynamic_pressure,  // [Pa]
		float air_temperature,  // [deg C]
		float humidity  // [%]
		);
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
		);
uint8_t BRIDGE_SendWindPkt(uint8_t p,
		float u,  // [m/s]
		float v,  // [m/s]
		float w  // [m/s]
		);
uint8_t BRIDGE_SendIMUPkt(uint8_t p, 
		float ax, float ay, float az, 
		float gx, float gy, float gz, 
		float mx, float my, float mz, float temp);
uint8_t BRIDGE_SendAccelPkt(uint8_t p, 
		float ax, float ay, float az, float temp);
uint8_t BRIDGE_SendGyroPkt(uint8_t p, 
		float gx, float gy, float gz, float temp);
uint8_t BRIDGE_SendMagPkt(uint8_t p,
		float mx, float my, float mz);
uint8_t BRIDGE_SendMagPkt_ID(uint8_t p, uint8_t node_id,
		float mx, float my, float mz);
uint8_t BRIDGE_SendActuatorPkt(uint8_t p, uint16_t *usec);
uint8_t BRIDGE_SendGNSSPkt(uint8_t p, 
		uint16_t week,
		uint8_t hours, uint8_t minutes, float seconds, 
		double latitude, double longitude, float altitude, 
		float heading, float speed, float pdop, uint8_t satellites, uint8_t fix_type,
		float vx, float vy, float vz);

uint8_t BRIDGE_SendGNSSUTCWPkt(uint8_t p, 
		uint16_t week,
		uint8_t hours, uint8_t minutes, float seconds);
uint8_t BRIDGE_SendGNSSLLAPkt(uint8_t p, 
		double latitude, double longitude, float altitude);
uint8_t BRIDGE_SendGNSSVelPkt(uint8_t p, 
		float heading, float speed,
		float vx, float vy, float vz);
uint8_t BRIDGE_SendGNSSHealthPkt(uint8_t p, 
		float pdop, uint8_t satellites, uint8_t fix_type);

uint8_t BRIDGE_SendGNSSRTCMPkt(uint8_t p, uint8_t size, uint8_t * payload);
uint8_t BRIDGE_SendGNSSSVINPkt(uint8_t p,
		uint32_t time_elapsed,
		uint32_t time_minimum,
		float accuracy,
		float accuracy_minimum,
		uint8_t flags);

uint8_t BRIDGE_SendSupplyPkt(uint8_t p, float voltage, float current, float coulomb_count, float temperature);

uint8_t BRIDGE_SendSupplyPkt_Old(uint8_t p, uint16_t voltage, uint16_t current);

uint8_t BRIDGE_SendNDVIPkt(uint8_t p, uint8_t id, float red, float near_ir, float ir_ambient, float ir_object);
uint8_t BRIDGE_SendAGLPkt(uint8_t p, float *ts, float *distance, float *velocity);
uint8_t BRIDGE_SendProximityPkt(uint8_t p, float ts, float distance, float velocity);

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
		uint16_t squawk);

uint8_t BRIDGE_SendTriggerPkt(uint8_t p, float *ts,
		uint16_t id,
		uint8_t channel);

uint8_t BRIDGE_SendDeployTubePkt(uint8_t p,
		uint8_t state,
		uint8_t parachute_door,
		uint8_t batt_voltage,
		uint8_t error);

uint8_t BRIDGE_SendDeployTubeCmdPkt(uint8_t p,
		uint8_t id,
		float value);

uint8_t BRIDGE_SendRIDPacket(uint8_t p,
		uint8_t aircraft_type,
		uint8_t base_mode,
		uint8_t state,
		uint8_t autopilot_type);

uint8_t BRIDGE_SendGCSLocation(uint8_t p,
		double latitude,
		double longitude,
		float altitude);

uint8_t BRIDGE_SendSerialNumber(uint8_t p,
		char serial_number[20]);

uint8_t BRIDGE_SendArmRemoteID(uint8_t p,
		uint8_t armed);

uint32_t BRIDGE_GetPktDrop(void);
/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif
  
#endif /* __BRIDGE_H */
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

