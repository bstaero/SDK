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

uint8_t BRIDGE_SendSupplyPkt(uint8_t p, float voltage, float current, float coulomb_count, float temperature);

uint8_t BRIDGE_SendSupplyPkt_Old(uint8_t p, uint16_t voltage, uint16_t current);

uint8_t BRIDGE_SendNDVIPkt(uint8_t p, uint8_t id, float red, float near_ir, float ir_ambient, float ir_object);
uint8_t BRIDGE_SendAGLPkt(uint8_t p, float *ts, float *distance, float *velocity);
uint8_t BRIDGE_SendProximityPkt(uint8_t p, float ts, float distance, float velocity);


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

