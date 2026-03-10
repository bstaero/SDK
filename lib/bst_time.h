/*=+--+=#=+--         SwiftCore Flight Management Software        --+=#=+--+=#*\
|               Copyright (C) 2025 Black Swift Technologies LLC.               |
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
#ifndef BST_TIME_H
#define BST_TIME_H

#include <stddef.h>
#include <time.h>
#include <signal.h>

#define MILLI2NANO  1000000.0f
#define SEC2NANO    1000000000.0f
#define SEC2MICRO   1000000.0f

#define HZ2MILLI(hz) (1000.0f / (float)(hz))

class TimeStamp {
	public:
		TimeStamp(const size_t s = 0, const size_t us = 0);
		~TimeStamp();

		void stamp();

		bool operator == (const TimeStamp & t) const;
		bool operator != (const TimeStamp & t) const;
		bool operator >  (const TimeStamp & t) const;
		bool operator <  (const TimeStamp & t) const;

		TimeStamp & operator =  (const TimeStamp & t);
		TimeStamp & operator =  (const long & t);
		TimeStamp & operator += (const TimeStamp & m);
		TimeStamp & operator -= (const TimeStamp & m);
		TimeStamp & operator += (const long & m);
		TimeStamp & operator -= (const long & m);

		TimeStamp operator + (const long & m) const;
		TimeStamp operator - (const long & m) const;
		long operator + (const TimeStamp & a) const;
		long operator - (const TimeStamp & a) const;

		double diff(const TimeStamp & a) const;
		double time() const;
		double secOfDay() const;

		size_t get_seconds() const;
		size_t get_usec() const;

	private:
		size_t seconds;
		size_t usec;
};

class IntervalTimer {
	public:
		IntervalTimer(float rate = 0, struct sigevent * sePtr = NULL);
		~IntervalTimer();

		void start();
		void stop();

		void setRunRate(float rate);
		float getRunRate() const;

	private:
		float run_rate;

#ifdef __linux__
		timer_t timer_id;
		bool timer_created;
#endif
		struct sigevent sev;
		bool has_custom_event;
};

#endif
