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
 * timeLib.cxx
 *
 * PURPOSE:
 *	Provides a class for managing a time
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

#ifdef VXWORKS
# include "vxWorks.h"
#endif

#include <signal.h>
#include <unistd.h>  // for gettimeofday()
#include <stdio.h>   // printf

#include "timeLib.h"

//-------------------------------------------------------------------- 
// TimeStamp
//-------------------------------------------------------------------- 
TimeStamp::TimeStamp( const size_t s, const size_t u )
{
	seconds = s;
	usec = u;
}


TimeStamp::~TimeStamp() 
{
	;
}

bool TimeStamp::operator == (const TimeStamp& t)
{
	return (	this->seconds == t.seconds && this->usec == t.usec );
}

bool TimeStamp::operator != (const TimeStamp& t)
{
	return !(this->seconds == t.seconds && this->usec == t.usec );
}

bool TimeStamp::operator > (const TimeStamp& t)
{
	return ( this->time() > t.time() );
}

bool TimeStamp::operator < (const TimeStamp& t)
{
	return ( this->time() < t.time() );
}

TimeStamp& TimeStamp::operator = (const TimeStamp& t)
{
	this->seconds = t.seconds;
	this->usec = t.usec;

	return *this;
}

TimeStamp& TimeStamp::operator = (struct tm & t)
{
	// get seconds from t
	seconds = mktime( &t );
	//if( seconds == -1 )
		//perror("mktime");

	//struct tm tl = t;
	//tl.tm_sec = 0;
	//tl.tm_min = 0;
	//tl.tm_hour = 0;

	usec = 0;

	return *this;
}

TimeStamp& TimeStamp::operator = (const long& t)
{
	seconds = (t / 1000000);
	usec = (t % 1000000);

	return *this;
}

TimeStamp& TimeStamp::operator += (const TimeStamp& a)
{
        int carry = 0;

        long udiff = (usec + a.usec);
        if( udiff < 0 )
                carry =  -1 + (int)(udiff / 1000000);
        else
                carry = (int)(udiff / 1000000);

        usec = (size_t)(udiff - carry * 1000000);
        seconds += (size_t)(carry) + a.seconds;

        return *this;
}

TimeStamp& TimeStamp::operator -= (const TimeStamp& a)
{
        int carry = 0;

        long udiff = (usec - a.usec);
        if( udiff < 0 )
                carry =  -1 + (int)(udiff / 1000000);
        else
                carry = (int)(udiff / 1000000);

        usec = (size_t)(udiff - carry * 1000000);
        seconds += (size_t)(carry) - a.seconds;

        return *this;
}

TimeStamp& TimeStamp::operator += (const long& m) 
{
	long udiff = (usec + m);
	int carry = 0;
	if( udiff < 0 ) 
		carry =  -1 + (int)(udiff / 1000000);
	else 
		carry = (int)(udiff / 1000000);

	seconds += (size_t)(carry);
	usec = (size_t)(udiff - carry * 1000000);

	return *this;
}
TimeStamp& TimeStamp::operator -= (const long& m)  {
	return *this += (-m);
}

void TimeStamp::stamp() 
{
#if !defined (_POSIX_TIMERS) || defined (__MACH__)
	struct timeval current;

	// sg_timestamp currtime;
	if( gettimeofday(&current, NULL) != OK )
		perror("TimeStamp::stamp - gettimeofday()");
	else {
		// timezone is an extern value in time.h and is number
		// of seconds from UTC
		seconds = current.tv_sec;// + timezone;
		usec = current.tv_usec;
	}

	//printf( "gettimeofday\n");
#else
	struct timespec current;
	(void)clock_gettime(CLOCK_REALTIME, &current);
	seconds = current.tv_sec;
	usec = current.tv_nsec / 1000;

	//printf( "clock_gettime: usec=%li seconds=%li\n",usec,seconds);
#endif
}


// get the fractional number of seconds 
double TimeStamp::time() const
{
	 return seconds + (double)(usec) / SEC2MICRO; 
} 

// increment the time stamp by the number of microseconds (usec)
TimeStamp TimeStamp::operator + (const long& m) 
{
	TimeStamp temp(*this);
	long udiff = (temp.usec + m);
	int carry = 0;
	if( udiff < 0 ) 
		carry =  -1 + (int)(udiff / 1000000);
	else 
		carry = (int)(udiff / 1000000);

	temp.seconds += (size_t)(carry);
	temp.usec = (size_t)(udiff - carry * 1000000);

	return temp;
}
TimeStamp TimeStamp::operator - (const long& m)  {
	return *this + (-m);
}

/*
TimeStamp TimeStamp::operator + (const TimeStamp& a) 
{
	TimeStamp temp(*this);
	int carry = 0;

	long udiff = (temp.usec + a.usec);
	if( udiff < 0 ) 
		carry =  -1 + (int)(udiff / 1000000);
	else 
		carry = (int)(udiff / 1000000);

	temp.usec = (size_t)(udiff - carry * 1000000);
	temp.seconds += (size_t)(carry) + a.seconds;

	return temp;
}
TimeStamp TimeStamp::operator - (const TimeStamp& a)  
{

	TimeStamp temp(*this);
	int carry = 0;

	long udiff = (temp.usec - a.usec);
	if( udiff < 0 ) 
		carry =  -1 + (int)(udiff / 1000000);
	else 
		carry = (int)(udiff / 1000000);

	temp.usec = (size_t)(udiff - carry * 1000000);
	temp.seconds += (size_t)(carry) - a.seconds;

	return temp;
}
*/

// difference between time stamps in microseconds (usec)
/*
long operator - (const TimeStamp& a, const TimeStamp& b)
{
	if( (a.seconds >= b.seconds) && (a.usec >= b.usec) )
		return (long)(SEC2MICRO * (a.seconds - b.seconds) + (a.usec - b.usec));
	else if( (a.seconds >= b.seconds) && (a.usec < b.usec) )
		return (long)(SEC2MICRO * (a.seconds - b.seconds - 1) + (SEC2MICRO + a.usec - b.usec));
	else 
		return (long)(SEC2MICRO * (a.seconds - b.seconds ) + ( a.usec - b.usec));
}
*/

double TimeStamp::diff(const TimeStamp& b)
{
	return (this->time() - b.time());
}

long TimeStamp::operator - (const TimeStamp& b)
{
	if( (seconds >= b.seconds) && (usec >= b.usec) )
		return (long)(SEC2MICRO * (seconds - b.seconds) + (usec - b.usec));
	else if( (seconds >= b.seconds) && (usec < b.usec) )
		return (long)(SEC2MICRO * (seconds - b.seconds - 1) + (SEC2MICRO + usec - b.usec));
	else 
		return (long)(SEC2MICRO * (seconds - b.seconds ) + ( usec - b.usec));
}

long TimeStamp::operator + (const TimeStamp& b)
{
	return (long)(SEC2MICRO * (seconds + b.seconds ) + ( usec + b.usec));
}

//-------------------------------------------------------------------- 
// Interval Timer
//-------------------------------------------------------------------- 

// constructor with hertz specified rate
IntervalTimer::IntervalTimer(float rate, struct sigevent *sePtr)
#if defined (_POSIX_TIMERS) & !defined (__MACH__)
  : timer(0)
#endif
{
	double Isec;

	runRate = rate;
	sec = 0;
  usec = 0;

#if defined (_POSIX_TIMERS) & !defined (__MACH__)
	if ( timer_create(CLOCK_REALTIME, sePtr, &timer) < 0 ) {
		perror("IntervalTimer::IntervalTimer - timer_create");
  }
#endif

	if( runRate == 0)
		Isec = 0;
	else	
		Isec = 1 / rate;

	if( runRate <= 0)
		return;
	else if( runRate <= 1) {
		 sec = int(Isec);
		 usec = int( (Isec - sec)*1000000.0 );
	} else {
		 sec = 0;
		 usec = int( Isec*1000000.0 );
	}

#ifdef DEBUG_PRINT
	printf("IntervalTimer::IntervalTimer -- sec=%i usec=%i\n",(int)sec,(int)usec);
#endif
}

void IntervalTimer::setRunRate(float rate)
{
	float Isec;

#ifdef _POSIX_TIMERS
	if( runRate > 0 ) {
    stop();
	}
#endif

	if( rate < 0 )
		return;

	runRate = rate;
	if( runRate == 0)
		Isec = 0;
	else	
		Isec = 1 / rate;

	if( runRate <= 1) {
		 sec = int(Isec);
		 usec = int( (Isec - sec)*1000000.0 );
	} else {
		 sec = 0;
		 usec = int( Isec*1000000.0 );
	}

#ifdef _POSIX_TIMERS
	if( runRate > 0 ) {
    start();
	}
#endif

}


IntervalTimer::~IntervalTimer()
{
#if !defined (_POSIX_TIMERS) || defined (__MACH__)
	// set timer to original settings 
	setitimer(ITIMER_REAL, &old_itimer, &itimer);
#else
	if( timer >= 0 ) {
		stop();
		timer_delete(timer);
	}
#endif

}

void IntervalTimer::stop()
{
#if !defined (_POSIX_TIMERS) || defined (__MACH__)
	itimer.it_interval.tv_sec = 0;
	itimer.it_interval.tv_usec = 0;
	itimer.it_value.tv_sec = 0;
	itimer.it_value.tv_usec = 0;

	// set timer to original settings 
	setitimer(ITIMER_REAL, &itimer, NULL);
#else
	itimer.it_interval.tv_sec = 0;
	itimer.it_interval.tv_nsec = 0;
	itimer.it_value.tv_sec = 0;
	itimer.it_value.tv_nsec = 0;

	if( timer < 0 )
		return;
	else if( timer_settime(timer, 0, &itimer, NULL) < 0)
		perror("IntervalTimer::stop - timer_settime");
#endif

}

void IntervalTimer::start()
{
#if !defined (_POSIX_TIMERS) || defined (__MACH__)
	// set itimervalues
	itimer.it_interval.tv_sec = sec;
	itimer.it_interval.tv_usec = usec;
	itimer.it_value.tv_sec = sec;
	itimer.it_value.tv_usec = usec;

	// start timer
	setitimer(ITIMER_REAL, &itimer, &old_itimer);
#else
	itimer.it_interval.tv_sec 	= sec;
	itimer.it_interval.tv_nsec 	= usec * 1000;
	itimer.it_value.tv_sec 		= sec;
	itimer.it_value.tv_nsec 	= usec * 1000;

	if( timer < 0 )
		printf("IntervalTimer::start - timer has not been created");
	else if( timer_settime(timer, 0, &itimer, NULL) < 0)
		perror("IntervalTimer::start - timer_settime");
#endif

#ifdef DEBUG_PRINT
	printf("IntervalTimer::start -- timer started\n");
#endif

}

