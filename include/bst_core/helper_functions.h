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

#ifndef _HELPER_FUNCTIONS_H_
#define _HELPER_FUNCTIONS_H_

#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
  #include "structs.h"
#endif

/*-----[ Math Functions ]--------*/

#ifndef sq
#define sq(x)  ((x)*(x))
#endif

#ifndef SIGN
#define SIGN(x) ( ((x) >= 0) - ((x) <= 0) )
#endif

// This one outputs 1 for x == 0
#ifndef SIGN1
#define SIGN1(x) ( ((x) >= 0) - ((x) < 0) )
#endif

static inline float deltaT(float _time, float _now) {

	// first check for properly initialized values
	if( _time < 0 || _now < 0 )
		return 0;

	// if we have a timer overun we need to be able to calculate
	// an accurate deltaT. 

	// FIXME - this is all bad, with single precision IEEE Float, by the
	// time we get here we are already more than 126 seconds off. Thus
	// for us to even worry about this, we need to move to double precision
	//if(_time > _now)
	//return ((_now - _time) + 0xFFFFFFFF) + 1.0f; 
	//else

	return (_now - _time);
}

#ifdef __cplusplus
float checkLimit(float value, const Limit_t * const limit); 
float checkLimit(float value, float limit_val);
float checkLimit(float value, float limit_min, float limit_max);
#endif

/*-----[ Types ]--------*/
#ifndef __cplusplus
  #include <stdbool.h>
#endif

#define CCMRAM __attribute__((section(".ccmram")))

#define DEG_TO_RADL  0.017453292519943295769236907684886 // M_PI/180.0
#define RAD_TO_DEGL  57.2957795131 // 180.0/M_PI

#define DEG_TO_RAD  0.017453292519943295769236907684886f // M_PI/180.0
#define RAD_TO_DEG  57.2957795131f // 180.0/M_PI

#define DYNP_TO_PA 3920.0f
#define KPH_TO_MPS 0.277777778f

#define LAT_TO_M(d_LAT)     ((d_LAT) * 111120.0)
#define M_TO_LAT(METERS)    ((METERS) / 111120.0)
#define LON_TO_M(d_LON,LAT)  ((d_LON) / 0.000009 * cosf((LAT)*DEG_TO_RAD))
#define M_TO_LON(METERS,LAT) ((METERS) * 0.000009 / (cosf((LAT)*DEG_TO_RAD)))

#define FT_TO_M 0.3048f
#define INHG_TO_PA 3386.85f
#define INHG_TO_HPA 33.8685f
#define INHG_TO_KPA 3.38685f

#define PSF_TO_PA 47.88020833333f
#define PSI_TO_PA 4136.85438f

#define MPH_TO_MPS 0.44704f
#define KTS_TO_MPS 0.514444444f

#define LBF_TO_N 4.448222f
#define LB_TO_KG 0.45359237f

#define SEC_TO_MSEC 1000.0f
#define SEC_TO_USEC 1000000.0f
#define SEC_TO_NSEC 1000000000.0f

#define RHO 1.06f

/*-----[ Constants ]--------*/

// Atmospheric Model
#define CONST_Tb 288.15f
#define CONST_Lb -0.0065f
#define CONST_Rb 8.31432f
#define CONST_M 0.0289644f
#define CONST_G 9.80665f

#ifdef __cplusplus
extern "C" {
#endif

/*-----[ Bitfields ]--------*/
bool CheckBits(uint8_t val, uint8_t bits);
uint8_t SetBits(uint8_t val, uint8_t bits);
uint8_t ClearBits(uint8_t val, uint8_t bits);

/*-----[ Rotation ]--------*/

void inertialToBody(float * x_b,float * y_b,float * z_b,float x_i,float y_i,float z_i,float phi,float theta,float psi);
void bodyToInertial(float * x_i,float * y_i,float * z_i,float x_b,float y_b, float z_b, float phi, float theta, float psi);

/*
void inertialToBodyQuat(float * x_b,float * y_b,float * z_b,float x_i,float y_i,float z_i,float * q);
*/

float angle2heading(float angle);
float heading2angle(float heading);

float angleDifference(float a2, float a1); // a2 - a1
float angleWrap(float angle);

/*-----[ Noise ]--------*/

float uniformDist(void);
float normalDist(float mean, float std);

/*-----[ Endianness ]--------*/

float changeEndiannessFloat (float value);
int16_t changeEndiannessInt16 (int16_t value);
uint16_t changeEndiannessUint16 (uint16_t value);
uint32_t changeEndiannessUint32 (uint32_t value);
uint64_t changeEndiannessUint64 (uint64_t value);

/*-----[ Checksum ]--------*/

bool checkFletcher16(const uint8_t * const data, uint8_t size);
void setFletcher16 (uint8_t * const data, uint8_t size);

/*-----[ Quaternion Conversion ]-----*/

float quat_to_roll(const float V[4]);
float quat_to_pitch(const float V[4]);
float quat_to_yaw(const float V[4]);

/*-----[ Filters ]-----*/
float FILT_1stOrderLP(float y, float x, float alpha);
float FILT_1stOrderLP_ComputeAlpha(float f_cutoff, float f_sample);
float FILT_RunningMean(float mean_prev, float x, uint32_t n);
void FILT_RunningMeanVariance(float mean_prev, float var_prev, float x, uint32_t n, float *mean, float *var);

float rate_limit(float curr_val, float desired_val, float rate_limit);

#ifdef __cplusplus
}
#endif

#endif
