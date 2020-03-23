#include "helper_functions.h"
#include <math.h>
#include <stdlib.h>

bool CheckBits(uint8_t val, uint8_t bits) {
	return ( (val & bits) == bits );
}

uint8_t SetBits(uint8_t val, uint8_t bits) {
	return (val | bits);
}

uint8_t ClearBits(uint8_t val, uint8_t bits) {
	return (val & ~bits);
}


void inertialToBody(float * x_b,float * y_b,float * z_b,float x_i,float y_i,float z_i,float theta,float phi,float psi) {
	*x_b = -(cosf(theta)*cosf(psi))*x_i \
			 -(cosf(psi)*sinf(theta)*sinf(phi) - cosf(theta)*sinf(psi))*y_i \
			 -(cosf(theta)*cosf(psi)*sinf(phi) + sinf(theta)*sinf(psi))*z_i;
	*y_b = -(cosf(phi)*sinf(psi))*x_i \
			 -(cosf(theta)*cosf(psi) + sinf(theta)*sinf(phi)*sinf(psi))*y_i \
			 -(-cosf(psi)*sinf(theta) + cosf(theta)*sinf(phi)*sinf(psi))*z_i;
	*z_b = (-sinf(phi))*x_i \
			 +(cosf(phi)*sinf(theta))*y_i \
			 +(cosf(theta)*cosf(phi))*z_i;
}

/* NOT USE AT THIS TIME
void inertialToBodyQuat(float * x_b,float * y_b,float * z_b,float x_i,float y_i,float z_i,float * q) {
	*x_b = (1 - 2*q[2]*q[2] - 2*q[3]*q[3])*x_i \
			 +(2*q[1]*q[2] - 2*q[3]*q[0])*y_i \
			 +(2*q[1]*q[3] + 2*q[2]*q[0])*z_i;
	*y_b = (2*q[1]*q[2] + 2*q[3]*q[0])*x_i \
			 +(1 - 2*q[1]*q[1] - 2*q[3]*q[3])*y_i \
			 +(2*q[2]*q[3] - 2*q[1]*q[0])*z_i;
	*z_b = (2*q[1]*q[3] - 2*q[2]*q[0])*x_i \
			 +(2*q[2]*q[3] + 2*q[1]*q[0])*y_i \
			 +(1 - 2*q[1]*q[1] - 2*q[2]*q[2])*z_i;
}
*/


void bodyToInertial(float * x_i,float * y_i,float * z_i,float x_b,float y_b, float z_b, float theta, float phi, float psi) {
	*x_i = (cosf(theta)*cosf(psi))*x_b \
			 +(-sinf(phi))*y_b \
			 +(cosf(phi)*sinf(psi))*z_b;

	*y_i = (cosf(psi)*sinf(theta)*sinf(phi) - cosf(theta)*sinf(psi))*x_b \
			 +(cosf(theta)*cosf(psi) + sinf(theta)*sinf(phi)*sinf(psi))*y_b \
			 +(cosf(phi)*sinf(theta))*z_b;

	*z_i = (cosf(theta)*cosf(psi)*sinf(phi) + sinf(theta)*sinf(psi))*x_b \
			 +(-cosf(psi)*sinf(theta) + cosf(theta)*sinf(phi)*sinf(psi))*z_b \
			 +(cosf(theta)*cosf(phi))*y_b;
}

float angle2heading(float angle) {
	float heading = fmod(angle+M_PI_2,2*M_PI);
	if(heading < 0) heading = heading+2*M_PI;
	return heading;
}

float heading2angle(float heading) {
	return (heading - M_PI_2);
}

// a2 - a1
float angleDifference(float a2, float a1) {
	float error = a2 - a1;
	if(error > M_PI) {
		while(error > M_PI) 
			error -= 2*M_PI;
	}
	else if(error < -M_PI) {
		while(error < -M_PI) 
			error += 2*M_PI;
	}

	return error;
}

float angleWrap(float angle) 
{
	if( angle > M_PI )
		return angle - 2*M_PI;
	else if( angle < -M_PI )
		return angle + 2*M_PI;

	return angle;
}

float checkLimit(float value, const Limit_t * const limit) 
{
	// check for valid limit
	if(limit == NULL || limit->max < limit->min) 
		return value;

	if(limit->max != INFINITY && value > limit->max)
		return limit->max;
	else if(limit->min != -INFINITY && value < limit->min)
		return limit->min;

	return value;
}

float checkLimit(float value, float limit_val) 
{
	// check for valid limit
	if(limit_val < 0 ) return value;

	if( value > limit_val ) return limit_val;
	else if( value < -limit_val ) return -limit_val;

	return value;
}

float uniformDist(){
	return ((float)rand() / (float)(RAND_MAX));
}

float normalDist(float mean, float std){
	return std*sqrtf( -2*logf( uniformDist() ) ) * cosf(2*M_PI*uniformDist()) + mean;
}

float changeEndiannessFloat (float value) {
	float temp;
	uint8_t * to = (uint8_t*)&temp;
	uint8_t * from = (uint8_t*)&value;

	for(uint8_t i=0; i<sizeof(float); i++)
		to[i] = from[sizeof(float)-i-1];

	return temp;
}

int16_t changeEndiannessInt16 (int16_t value) {
	return (value << 8) | (value >> 8);
}

uint16_t changeEndiannessUint16 (uint16_t value) {
	return (value << 8) | (value >> 8);
}

uint32_t changeEndiannessUint32 (uint32_t value) {
	uint32_t temp;
	uint8_t * to = (uint8_t*)&temp;
	uint8_t * from = (uint8_t*)&value;

	for(uint8_t i=0; i<sizeof(uint32_t); i++)
		to[i] = from[sizeof(uint32_t)-i-1];

	return temp;
}

uint64_t changeEndiannessUint64 (uint64_t value) {
	uint64_t temp;
	uint8_t * to = (uint8_t*)&temp;
	uint8_t * from = (uint8_t*)&value;

	for(uint8_t i=0; i<sizeof(uint64_t); i++)
		to[i] = from[sizeof(uint64_t)-i-1];

	return temp;
}

#include <stdio.h>

/**
 * @brief  Verify checksum using all but the last two bytes of data
 * @param  data Pointer to the data
 * @param  size Size of the data, including the checksum bytes
 * @retval 1-Valid, 0-Invalid
 */
bool checkFletcher16(const uint8_t * const data, uint8_t size) {
	uint16_t sum1 = 0;
	uint16_t sum2 = 0;

	for( int i = 0; i < size; i++ ) {
		sum1 = (sum1 + data[i]) % 255;
		sum2 = (sum2 + sum1) % 255;
	}

	return ((sum2 << 8) | sum1) == 0;
}

/**
 * @brief  Generate checksum using all but the last two bytes of data
 * @param  data Pointer to the data
 * @param  size Size of the data, including space allocated for the checksum bytes
 * @retval None
 */
void setFletcher16 (uint8_t * const data, uint8_t size){
	uint16_t sum1 = 0;
	uint16_t sum2 = 0;

	for( int i = 0; i < (size-2); i++ ) {  // excludes checksum bytes
		sum1 = (sum1 + data[i]) % 255;
		sum2 = (sum2 + sum1) % 255;
	}

	data[size-2] = 255 - (( sum1 + sum2 ) % 255);
	data[size-1] = 255 - (( sum1 + data[size-2] ) % 255);

	//uint16_t checksum1 = 255 - (( sum1 + sum2 ) % 255);
	//uint16_t checksum2 = 255 - (( sum1 + checksum1 ) % 255);

	//return ((checksum2 << 8) | checksum1);
}

/**
 * @brief  Convert quaternion to roll angle
 * @param  data pointer to quaternion
 * @retval roll angle in radians
 */
float quat_to_roll(const float V[4]) {
	return atan2f( 2*(V[2]*V[3] + V[0]*V[1]), V[0]*V[0] - V[1]*V[1] - V[2]*V[2] + V[3]*V[3] );
}

/**
 * @brief  Convert quaternion to pitch angle
 * @param  data pointer to quaternion
 * @retval pitch angle in radians
 */
float quat_to_pitch(const float V[4]) {
	return asinf( -2*(V[1]*V[3] - V[0]*V[2]) );
}

/**
 * @brief  Convert quaternion to yaw angle
 * @param  data pointer to quaternion
 * @retval yaw angle in radians
 */
float quat_to_yaw(const float V[4]) {
	return atan2f( 2*(V[1]*V[2] + V[0]*V[3]), V[0]*V[0] + V[1]*V[1] - V[2]*V[2] - V[3]*V[3] );
}

/**
 * @brief Compute running average
 * @param[in] y old and new filtered data
 * @param[in] x new sample to be filtered
 * @param[in] alpha smoothing factor, i.e. (sample_time)/(RC + sample_time)
 * @retval 1st Order Low-Pass filtered result.
 */
float FILT_1stOrderLP(float y, float x, float alpha)
{
	return (y + alpha*(x - y));
}

/**
 * @brief Compute alpha smoothing factor, i.e. (sample_time)/(RC + sample_time)
 * @param[in] f_cutoff cut-off frequency [Hz]
 * @param[in] f_sample sampling frequency [Hz]
 * @retval 1st Order Low-Pass smoothing factor, alpha.
 */
float FILT_1stOrderLP_ComputeAlpha(float f_cutoff, float f_sample)
{
	float t_sample = 1.0f/f_sample; // [sec]
	float rc = (float)(1.0f/(2*M_PI*f_cutoff)); // [rad/s]
	return t_sample/(rc + t_sample);
}

/**
 * @brief Compute running mean
 * @param[in] mean_prev running mean
 * @param[in] x new sample to be included in the mean
 * @param[in] n total samples thus far
 * @retval Running mean.
 */
float FILT_RunningMean(float mean_prev, float x, uint32_t n)
{
	return mean_prev + (x - mean_prev)/n;
}

/**
 * @brief Compute running mean and variance
 * @param[in] mean_prev running mean at t-1
 * @param[in] var_prev running variance at t-1
 * @param[in] x new sample to be included in the average at t
 * @param[in] n total samples thus far at t
 * @param[out] mean at t
 * @param[out] variance at t
 * @retval Running average.
 */
void FILT_RunningMeanVariance(float mean_prev, float var_prev, float x, uint32_t n, float *mean, float *var)
{
	float delta = x - mean_prev;

	*mean = mean_prev + delta/n;
	*var = (var_prev*(n-1) + delta*(x - mean_prev))/n;
}


float rate_limit(float curr_val, float desired_val, float rate_limit)
{
	if( fabs(desired_val - curr_val) > rate_limit ) {
		return ( curr_val + rate_limit * SIGN(desired_val - curr_val) );
	} 

	return desired_val;
}

bool InvertMatrix4x4(const float m[4][4], float invOut[4][4])
{
	float inv[4][4], det;
	uint8_t i,j;

	inv[0][0] = m[1][1]  * m[2][2] * m[3][3] - 
		m[1][1]  * m[2][3] * m[3][2] - 
		m[2][1]  * m[1][2]  * m[3][3] + 
		m[2][1]  * m[1][3]  * m[3][2] +
		m[3][1] * m[1][2]  * m[2][3] - 
		m[3][1] * m[1][3]  * m[2][2];

	inv[1][0] = -m[1][0]  * m[2][2] * m[3][3] + 
		m[1][0]  * m[2][3] * m[3][2] + 
		m[2][0]  * m[1][2]  * m[3][3] - 
		m[2][0]  * m[1][3]  * m[3][2] - 
		m[3][0] * m[1][2]  * m[2][3] + 
		m[3][0] * m[1][3]  * m[2][2];

	inv[2][0] = m[1][0]  * m[2][1] * m[3][3] - 
		m[1][0]  * m[2][3] * m[3][1] - 
		m[2][0]  * m[1][1] * m[3][3] + 
		m[2][0]  * m[1][3] * m[3][1] + 
		m[3][0] * m[1][1] * m[2][3] - 
		m[3][0] * m[1][3] * m[2][1];

	inv[3][0] = -m[1][0]  * m[2][1] * m[3][2] + 
		m[1][0]  * m[2][2] * m[3][1] +
		m[2][0]  * m[1][1] * m[3][2] - 
		m[2][0]  * m[1][2] * m[3][1] - 
		m[3][0] * m[1][1] * m[2][2] + 
		m[3][0] * m[1][2] * m[2][1];

	inv[0][1] = -m[0][1]  * m[2][2] * m[3][3] + 
		m[0][1]  * m[2][3] * m[3][2] + 
		m[2][1]  * m[0][2] * m[3][3] - 
		m[2][1]  * m[0][3] * m[3][2] - 
		m[3][1] * m[0][2] * m[2][3] + 
		m[3][1] * m[0][3] * m[2][2];

	inv[1][1] = m[0][0]  * m[2][2] * m[3][3] - 
		m[0][0]  * m[2][3] * m[3][2] - 
		m[2][0]  * m[0][2] * m[3][3] + 
		m[2][0]  * m[0][3] * m[3][2] + 
		m[3][0] * m[0][2] * m[2][3] - 
		m[3][0] * m[0][3] * m[2][2];

	inv[2][1] = -m[0][0]  * m[2][1] * m[3][3] + 
		m[0][0]  * m[2][3] * m[3][1] + 
		m[2][0]  * m[0][1] * m[3][3] - 
		m[2][0]  * m[0][3] * m[3][1] - 
		m[3][0] * m[0][1] * m[2][3] + 
		m[3][0] * m[0][3] * m[2][1];

	inv[3][1] = m[0][0]  * m[2][1] * m[3][2] - 
		m[0][0]  * m[2][2] * m[3][1] - 
		m[2][0]  * m[0][1] * m[3][2] + 
		m[2][0]  * m[0][2] * m[3][1] + 
		m[3][0] * m[0][1] * m[2][2] - 
		m[3][0] * m[0][2] * m[2][1];

	inv[0][2] = m[0][1]  * m[1][2] * m[3][3] - 
		m[0][1]  * m[1][3] * m[3][2] - 
		m[1][1]  * m[0][2] * m[3][3] + 
		m[1][1]  * m[0][3] * m[3][2] + 
		m[3][1] * m[0][2] * m[1][3] - 
		m[3][1] * m[0][3] * m[1][2];

	inv[1][2] = -m[0][0]  * m[1][2] * m[3][3] + 
		m[0][0]  * m[1][3] * m[3][2] + 
		m[1][0]  * m[0][2] * m[3][3] - 
		m[1][0]  * m[0][3] * m[3][2] - 
		m[3][0] * m[0][2] * m[1][3] + 
		m[3][0] * m[0][3] * m[1][2];

	inv[2][2] = m[0][0]  * m[1][1] * m[3][3] - 
		m[0][0]  * m[1][3] * m[3][1] - 
		m[1][0]  * m[0][1] * m[3][3] + 
		m[1][0]  * m[0][3] * m[3][1] + 
		m[3][0] * m[0][1] * m[1][3] - 
		m[3][0] * m[0][3] * m[1][1];

	inv[3][2] = -m[0][0]  * m[1][1] * m[3][2] + 
		m[0][0]  * m[1][2] * m[3][1] + 
		m[1][0]  * m[0][1] * m[3][2] - 
		m[1][0]  * m[0][2] * m[3][1] - 
		m[3][0] * m[0][1] * m[1][2] + 
		m[3][0] * m[0][2] * m[1][1];

	inv[0][3] = -m[0][1] * m[1][2] * m[2][3] + 
		m[0][1] * m[1][3] * m[2][2] + 
		m[1][1] * m[0][2] * m[2][3] - 
		m[1][1] * m[0][3] * m[2][2] - 
		m[2][1] * m[0][2] * m[1][3] + 
		m[2][1] * m[0][3] * m[1][2];

	inv[1][3] = m[0][0] * m[1][2] * m[2][3] - 
		m[0][0] * m[1][3] * m[2][2] - 
		m[1][0] * m[0][2] * m[2][3] + 
		m[1][0] * m[0][3] * m[2][2] + 
		m[2][0] * m[0][2] * m[1][3] - 
		m[2][0] * m[0][3] * m[1][2];

	inv[2][3] = -m[0][0] * m[1][1] * m[2][3] + 
		m[0][0] * m[1][3] * m[2][1] + 
		m[1][0] * m[0][1] * m[2][3] - 
		m[1][0] * m[0][3] * m[2][1] - 
		m[2][0] * m[0][1] * m[1][3] + 
		m[2][0] * m[0][3] * m[1][1];

	inv[3][3] = m[0][0] * m[1][1] * m[2][2] - 
		m[0][0] * m[1][2] * m[2][1] - 
		m[1][0] * m[0][1] * m[2][2] + 
		m[1][0] * m[0][2] * m[2][1] + 
		m[2][0] * m[0][1] * m[1][2] - 
		m[2][0] * m[0][2] * m[1][1];

	det = m[0][0] * inv[0][0] + m[0][1] * inv[1][0] + m[0][2] * inv[2][0] + m[0][3] * inv[3][0];

	// check for ill conditioned matrix
	if (det <= 1e-10)
		return false;

	det = 1.0 / det;

	for (i = 0; i < 4; i++)
		for (j = 0; j < 4; j++)
			invOut[i][j] = inv[i][j] * det;

	return true;
}

#ifdef TESTING
#ifdef TESTING_MAIN
 #define CATCH_CONFIG_MAIN
#endif

#include "catch.hpp"

SCENARIO("Helper Function: helpers", "[helpers]")
{
	GIVEN( "A singular 4x4 matrix" ) {
		float V[4][4] = {
			{1,     2,     3,     4}, 
			{4,     5,     6,     7}, 
			{8,     9,    10,    11},
			{12,   13,    14,    15} };

		REQUIRE( V[0][3] == 4 );
		REQUIRE( V[3][0] == 12 );

		WHEN( "the inverse is taken ") {
			float invV[4][4];
			bool nonSingular = InvertMatrix4x4( V,  invV );

			THEN( "then it should not be solvable" ) {
				REQUIRE( nonSingular == false );
			}
		}
	}


	GIVEN( "A non-singular 4x4 matrix" ) {
		float V[4][4] = {
			{64.0000,  -0.0000,    0.0000,   0},
			{-0.0000,   0.8712,   -0.0000,  -0.0000},
			{0.0000,   -0.0000,    0.8712,   0},
			{0,        -0.0000,    0,        0.0164} };

		WHEN( "the inverse is taken ") {
			float invV[4][4];
			bool nonSingular = InvertMatrix4x4(V, invV );

			THEN( "then it should be solvable" ) {
				REQUIRE( nonSingular == true );
			}
			THEN( "then it should have the solution " ) {
				REQUIRE( invV[0][0] == Approx(0.015625) );
				REQUIRE( invV[0][1] == Approx(0.0) );
				REQUIRE( invV[0][2] == Approx(0.0) );
				REQUIRE( invV[0][3] == Approx(0.0) );

				REQUIRE( invV[1][0] == Approx(0.0) );
				REQUIRE( invV[1][1] == Approx(1.147842) );
				REQUIRE( invV[1][2] == Approx(0.0) );
				REQUIRE( invV[1][3] == Approx(0.0) );

				REQUIRE( invV[2][0] == Approx(0.0) );
				REQUIRE( invV[2][1] == Approx(0.0) );
				REQUIRE( invV[2][2] == Approx(1.147842) );
				REQUIRE( invV[2][3] == Approx(0.0) );

				REQUIRE( invV[3][0] == Approx(0.0) );
				REQUIRE( invV[3][1] == Approx(0.0) );
				REQUIRE( invV[3][2] == Approx(0.0) );
				REQUIRE( invV[3][3] == Approx(60.97561) );
			}
		}
	}
}

#endif
