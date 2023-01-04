/*=+--+=#=+--     Unmanned Aerial System Management Software      --+=#=+--+=#*\
|									Copyright (C) 2009 Jack Elston, Cory Dixon                   |
|                                                                              |
      This program is free software; you can redistribute it and/or             
      modify it under the terms of the GNU General Public License               
      as published by the Free Software Foundation; either version 2            
      of the License, or (at your option) any later version.                    
                                                                                
      This program is distributed in the hope that it will be useful,           
      but WITHOUT ANY WARRANTY; without even the implied warranty of            
      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             
      GNU General Public License for more details.                              
                                                                                
      You should have received a copy of the GNU General Public License         
      along with this program; if not, write to the Free Software               
      Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA                 
      02111-1307, USA.                                                          
                                                                                
 			  		Jack Elston                       Cory Dixon                        
|			  		elstonj@colorado.edu              dixonc@colorado.edu              |
|                                                                              |
\*=+--+=#=+--                 --+=#=+--+=#=+--                    --+=#=+--+=#*/

/***********************************************************************
 *
 * FILENAME:
 * timeLib.h
 *
 * PURPOSE:
 * Provides a class for managing a time
 *
 * CREATED:
 * 11/2000 by Cory Dixon
 *
 * LAST MODIFIED:
 * $Author: dixonc $
 * $Date: 2005/11/02 20:27:20 $
 * $Revision: 1.6 $
 *
 ***********************************************************************/

#ifndef __cplusplus
# error This library requires C++
#endif								   

#ifndef _TIMELIB_H
#define _TIMELIB_H

#define _POSIX_C_SOURCE 200809L

#ifndef VXWORKS
# include <sys/time.h>
#endif
#include <time.h>
#include "type_defs.h"

//
// Global Defines
// USE THESE VERY CAREFULLY
//
#define MILLI2NANO   1000000.0f
#define SEC2NANO     1000000000.0f
#define SEC2MICRO    1000000.0f
#define HZ2MILLI(hz) ( 1000.0f / (float)hz )

//--------------------------------------------------------------------
// TimeStamp
//--------------------------------------------------------------------

// forward declaration of friends.
//class TimeStamp;
//TimeStamp operator + (const TimeStamp& t, const long& m);
//long operator - (const TimeStamp& a, const TimeStamp& b);

// <summary> easy time stamps </summary>
//
// The TimeStamp class allows you to mark and compare time stamps
// with microsecond accuracy (if your system has support for this
// level of accuracy.)
// 
// The TimeStamp is useful for tracking the elapsed time of various
// events in your program. You can also use it to keep constistant
// motion across varying frame rates. The returned value for secOfDay()
// is only the seconds of the day. If you want the number of seconds
// from the standard computer epoch of 1/1/1970  use the get_seconds()
// function. If you want total fractional value of seconds and usec use
// time()
//
class TimeStamp {

private:
	unsigned long int seconds;		// seconds of day, in local time
	unsigned long int usec;		// microseconds of second
	//long int epochSeconds;	// seconds since 00:00:00, January 1, 1970 UTC
	//struct tm t;

public:

	//
	// This creates an instance of the TimeStamp object. When
	// calling the constructor you may provide initial seconds an
	// microseconds values.
	// s - initial seconds value
	// m - initial microseconds value
	// 
	TimeStamp( const size_t s = 0, const size_t m = 0 );
	~TimeStamp();

	// Update stored time to current time 
	void stamp();

	// Compare two time stamps for equality 
	bool operator == ( const TimeStamp& t );
	bool operator != ( const TimeStamp& t );
	bool operator > ( const TimeStamp& t );
	bool operator < ( const TimeStamp& t );

	// Set equal to
	TimeStamp& operator = ( const TimeStamp & t );
	TimeStamp& operator = ( struct tm & t );
	TimeStamp& operator = ( const long& t );

	// some microseconds
	TimeStamp& operator += (const TimeStamp& m);
	TimeStamp& operator -= (const TimeStamp& m);
	TimeStamp& operator += (const long& m);
	TimeStamp& operator -= (const long& m);

	// 
	// Increment time by the specified number of microseconds
	// input
	// 	m - microseconds increment
	// return 
 	//	new time stamp
	//
	TimeStamp operator + (const long& m);
	TimeStamp operator - (const long& m);

	//
	// Add/Subtract two time stamps returning number of microseconds
	//   input
	// 	a - timestamp 1
	// 	b - timestame 2
	// return 
	//	difference in microseconds
	//
	long operator + (const TimeStamp& a);
	long operator - (const TimeStamp& a);

	double diff(const TimeStamp& a); // returns seconds

	//
	// return the fractional seconds, using a 4-byte float
	//
	double time() const;
	double secOfDay() const;

	// return the private member values
	size_t get_seconds() { return seconds; }
	size_t get_usec() { return usec; }
	//size_t get_epochseconds() { return epochSeconds; }

/*
	String timeStr( char *format = "%a %b %e %T" ) {
		char buf[ 256 ];
		(void)strftime(buf, 255, format, localtime((time_t *)&epochSeconds) ); 
		return String(buf);
	}

	void print() const { 
		cout << "seconds=" << seconds << " usec=" << usec << endl; 
	}
*/
};

//--------------------------------------------------------------------
// Interval Timer
//--------------------------------------------------------------------

//
// <summary> Interval Timer </summary>
//
// The IntervalTimer class allows you to setup a interval timer
// with resolution set by the machine
// 
class IntervalTimer 
{
private:
#if !defined (_POSIX_TIMERS) || defined (__MACH__)
	struct itimerval itimer;
	struct itimerval old_itimer;
#else
	timer_t timer;
	struct itimerspec itimer, old_itimer;
#endif

	float runRate;	// Hertz value for interval
	size_t sec;	// number of seconds
	size_t usec;	// number of micro-seconds

public:

	//
	// Constructors
	//
	IntervalTimer(float rate = 0, struct sigevent *sePtr = NULL);
	~IntervalTimer();

	// start the timer 
	void start();
	void stop();

	void setRunRate(float rate);
	float getRunRate() { return runRate; }
};

#endif // _TIMESTAMP_H


