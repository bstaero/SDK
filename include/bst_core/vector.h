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

#ifndef VECTOR_H
#define VECTOR_H

#include <stdlib.h>
#include <assert.h>

class Vector
{
	protected:
		float *V;
		size_t len;

	public:
		// constructor
		Vector();
		Vector(size_t _len);
		Vector(const Vector& v);

		// destructor
		~Vector();

		// get vector
		void getV(float * const v) const;

		// status
		bool isFinite() const;

		// Equals operator
		Vector& operator = (const Vector& v);
		Vector& operator = (const float val);

		// compound assignment
		Vector& operator += (const Vector& v);
		Vector& operator -= (const Vector& v);
		Vector& operator += (const float val);
		Vector& operator -= (const float val);
		Vector& operator *= (const float val);
		Vector& operator /= (const float val);

		// Get and set values of the vector.
		float operator () (size_t ind) const { assert(ind < len); return V[ind]; };
		float& operator () (size_t ind) { assert(ind < len); return V[ind]; };

		// Return the vector length
		size_t length() const { return len; }
		void setSize(size_t _len);

		Vector operator - () const;

		// Scalar multiplication, division
		Vector operator * (const float val) const;
		friend Vector operator * (const float val, const Vector& v);
		Vector operator / (const float val) const;

		// vector dot product, addition, and subtraction
		friend float dot(const Vector& v1, const Vector& v2);
		friend Vector operator + (const Vector& v1, const Vector& v2);
		friend Vector operator - (const Vector& v1, const Vector& v2);
		friend Vector cross3(const Vector& v1, const Vector& v2);

		// Norm of the vector
		float norm() const;
		float norm2() const;
		float min() const;
		float max() const;

};

#endif
