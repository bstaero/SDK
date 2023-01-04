#include <math.h>
#include <float.h>
#include "debug.h"
#include "vector.h"

Vector::Vector()
{
	pmesg(VERBOSE_ALLOC, "Vector::Vector()\n");

	len = 0;
	V = NULL;
}

Vector::Vector(size_t _len)
{
	V = NULL;
	len = _len;
	if( len > 0 ) {
		pmesg(VERBOSE_ALLOC, "Vector::Vector(%u)\n",len);
		V = new float[len];
		for (size_t i=0; i<len; i++)
			V[i] = 0.0;
	}
}

Vector::Vector(const Vector& v)
{
	V = NULL;
	len = v.length();
	if( len > 0 ) {
		pmesg(VERBOSE_ALLOC, "Vector::Vector(Vector)\n");
		V = new float[len];
		for (size_t i=0; i<len; i++)
			V[i] = v.V[i];
	}
}

Vector::~Vector()
{
	if( V != NULL ) 
		delete [] V;
	V = NULL;
}

void Vector::getV(float * const v) const 
{
	if( v == NULL ) return;

	for (size_t i=0; i<len; i++)
		v[i] = this->V[i];
}

Vector& Vector::operator = (const Vector& v)
{
	if( len != v.length() ) {
		if( V != NULL )
			delete [] V;
		V = NULL;

		pmesg(VERBOSE_ALLOC, "Vector::op =(Vector) : this.len=%u v.len=%u\n",
				len, v.len);

		len = v.length();
		V = new float[len];
	}

	for (size_t i=0; i<len; i++)
		V[i] = v.V[i];

	return *this;
}

Vector& Vector::operator = (const float v)
{
	for (size_t i=0; i<len; i++)
		V[i] = v;

	return *this;
}


void Vector::setSize(size_t _len)
{
	if( len != _len ) {
		if( V != NULL ) 
			delete [] V;
		V = NULL;

		len = _len;

		pmesg(VERBOSE_ALLOC, "Vector::setSize(%u)\n",len);
		V = new float[len];
	}

	for (size_t i=0; i<len; i++)
		V[i] = 0.0f;
}

Vector Vector::operator - () const
{
	Vector tmp(*this);
	for (size_t i=0; i<len; i++)
		tmp.V[i] = -V[i];
	return tmp;
}

Vector Vector::operator * (float val) const
{
	Vector tmp(*this);
	for (size_t i=0; i<len; i++)
		tmp.V[i] *= val;
	return tmp;
}

Vector Vector::operator / (float val) const
{
	Vector tmp(*this);
	for (size_t i=0; i<len; i++)
		tmp.V[i] /= val;
	return tmp;
}

Vector operator * (float val, const Vector& v)
{
	Vector tmp(v);
	for (size_t i=0; i<v.length(); i++)
		tmp.V[i] *= val;
	return tmp;
}

Vector operator + (const Vector& v1, const Vector& v2)
{
	assert( v1.length() == v2.length());

	Vector tmp(v1.length());
	for (size_t i=0; i<v1.length(); i++)
		tmp.V[i] = v1.V[i] + v2.V[i];
	return tmp;
}

Vector operator - (const Vector& v1, const Vector& v2)
{
	assert( v1.length() == v2.length());

	Vector tmp(v1.length());
	for (size_t i=0; i<v1.length(); i++)
		tmp.V[i] = v1.V[i] - v2.V[i];
	return tmp;
}

Vector& Vector::operator += (const Vector& v)
{
	if( len != v.length() ) return *this;

	for (size_t i=0; i<len; i++)
		V[i] += v.V[i];

	return *this;
}

Vector& Vector::operator -= (const Vector& v)
{
	if( len != v.length() ) return *this;

	for (size_t i=0; i<len; i++)
		V[i] -= v.V[i];

	return *this;

}

Vector& Vector::operator += (const float val)
{
	for (size_t i=0; i<len; i++)
		V[i] += val;

	return *this;

}

Vector& Vector::operator -= (const float val)
{
	for (size_t i=0; i<len; i++)
		V[i] -= val;

	return *this;

}

Vector& Vector::operator *= (const float val)
{
	for (size_t i=0; i<len; i++)
		V[i] *= val;

	return *this;

}
Vector& Vector::operator /= (const float val)
{
	for (size_t i=0; i<len; i++)
		V[i] /= val;

	return *this;

}


float dot(const Vector& v1, const Vector& v2)
{
	assert(v1.length() == v2.length());

	float out = 0;
	for (size_t i=0; i<v1.length(); i++)
		out += v1.V[i]*v2.V[i];
	return out;
}

Vector cross3(const Vector& v1, const Vector& v2)
{
	assert(v1.length() == 3 && v2.length() == 3);

	Vector tmp(3);
	tmp.V[0] = v1.V[1]*v2.V[2] - v1.V[2]*v2.V[1];
	tmp.V[1] = v1.V[2]*v2.V[0] - v1.V[0]*v2.V[2];
	tmp.V[2] = v1.V[0]*v2.V[1] - v1.V[1]*v2.V[0];
	return tmp;
}

float Vector::norm() const
{
	float norm = 0.0f;
	for(size_t i=0; i<len; i++)
		norm += V[i]*V[i];
	return sqrtf(norm);
} 

float Vector::norm2() const
{
	float norm = 0.0f;
	for(size_t i=0; i<len; i++)
		norm += V[i]*V[i];
	return norm;
} 


bool Vector::isFinite() const {
	for(size_t i=0; i<len; i++)
		if( !isfinite(V[i]) )
			return false;

	return true;
}

float Vector::min() const
{
	float min = FLT_MAX;
	for(size_t i=0; i<len; i++) {
		if( V[i] < min ) {
			min = V[i];
		}
	}
	return min;
} 

float Vector::max() const
{
	float max = -FLT_MAX;
	for(size_t i=0; i<len; i++) {
		if( V[i] > max ) {
			max = V[i];
		}
	}
	return max;
}

#ifdef TESTING
#ifdef TESTING_MAIN
 #define CATCH_CONFIG_MAIN
#endif

#include "catch.hpp"

SCENARIO("VectorTest: vectors", "[vector]")
{
	GIVEN( "A vector with no items" ) {
		Vector v;

		REQUIRE( v.length() == 0 );

		WHEN( "the size is increased" ) {
			v.setSize( 10 );

			THEN( "the size changed" ) {
				REQUIRE( v.length() == 10 );
			}
		}
		WHEN( "the size is reduced" ) {
			v.setSize( 5 );

			THEN( "the size changed" ) {
				REQUIRE( v.length() == 5 );
			}
		}

			//THEN("should not be able to index beyond size") {
				//REQUIRE( v(2) == 2 );
			//}
	}

	GIVEN( "A two element vector " ) {
		Vector v(2);
		REQUIRE( v.length() == 2 );

		// set values
		v(0) = 1;
		v(1) = 2;

		WHEN("new elements are assigned") {
			THEN("the elements should be defined") {
				REQUIRE( v(0) == 1 );
				REQUIRE( v(1) == 2 );
			}
		}


		WHEN("a vector is assigned") {

			REQUIRE( v(0) == 1 );
			REQUIRE( v(1) == 2 );

			Vector v1;
			v1 = v;

			THEN("it should be the same size") {
				REQUIRE( v1.length() == 2 );
			}

			THEN("the elements should be same as original") {
				REQUIRE( v1(0) == 1 );
				REQUIRE( v1(1) == 2 );
			}
		}

		WHEN("a vector is constructecd from another vector") {
			Vector v1(v);

			THEN("it should be the same size") {
				REQUIRE( v1.length() == 2 );
			}

			THEN("the elements should be same as original") {
				REQUIRE( v1(0) == 1 );
				REQUIRE( v1(1) == 2 );
			}
		}
	}

	GIVEN( "A two element vector with values" ) {

		Vector v(2);
		REQUIRE( v.length() == 2 );

		// set values
		v(0) = 1;
		v(1) = 2;

		REQUIRE( v(0) == 1 );
		REQUIRE( v(1) == 2 );

		WHEN("the 2-norm is used") {
			float d = v.norm();
			THEN("it should be defined") {
				REQUIRE( d == sqrtf(1+4) );
			}
		}


		WHEN("two vectors are added to make new vector") {
			Vector v1 = v + v;
			THEN("ther sums should match") {
				REQUIRE( v1(0) == 2 );
				REQUIRE( v1(1) == 4 );
			}
		}

		WHEN("vector added to a vector") {
			Vector v1 = v;
			v1 += v;

			THEN("ther sums should match") {
				REQUIRE( v1(0) == 2 );
				REQUIRE( v1(1) == 4 );
			}
		}
	
		WHEN("two vectors are subtracted to make new vector") {
			Vector v1 = v - v;
			THEN("ther sums should match") {
				REQUIRE( v1(0) == 0 );
				REQUIRE( v1(1) == 0 );
			}
		}

		WHEN("vector substracted from a vector") {
			Vector v1 = v;
			v1 -= v;

			THEN("ther sums should match") {
				REQUIRE( v1(0) == 0 );
				REQUIRE( v1(1) == 0 );
			}
		}

		WHEN("vectors multiplied by value to make new vector") {
			Vector v1 = v * 5.0f;
			THEN("ther value should match") {
				REQUIRE( v1(0) == 5.0f );
				REQUIRE( v1(1) == 10.0f );
			}
		}

		WHEN("vector multiplied to a value") {
			Vector v1 = v;
			v1 *= 10.0f;

			THEN("ther value should match") {
				REQUIRE( v1(0) == 10.0f );
				REQUIRE( v1(1) == 20.0f );
			}
		}


		WHEN("vectors divided by value to make new vector") {
			Vector v1 = v / 5.0f;
			THEN("ther value should match") {
				REQUIRE( v1(0) == 1/5.0f );
				REQUIRE( v1(1) == 2/5.0f );
			}
		}

		WHEN("vector divided to a value") {
			Vector v1 = v;
			v1 /= 10.0f;

			THEN("ther value should match") {
				REQUIRE( v1(0) == 1/10.0f );
				REQUIRE( v1(1) == 2/10.0f );
			}
		}

		WHEN("dot product of two vectors ") {
			Vector v1 = v;
			float d = dot(v,v1);
			THEN("the value should match") {
				REQUIRE( d == 5 );
			}
		}

		WHEN("the vector is negated") {
			Vector v1 = -v;
			THEN("it should be negative") {
				REQUIRE( v1(0) == -1 );
				REQUIRE( v1(1) == -2 );
			}
		}
	}
}

#endif
