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
#include "bst_time.h"

#include <string.h>
#include <sys/time.h>

#ifdef __APPLE__
#include <mach/mach_time.h>
#endif

/* ---- TimeStamp ---- */

TimeStamp::TimeStamp(const size_t s, const size_t us)
	: seconds(s), usec(us)
{
}

TimeStamp::~TimeStamp() { }

void TimeStamp::stamp()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	seconds = (size_t)tv.tv_sec;
	usec    = (size_t)tv.tv_usec;
}

bool TimeStamp::operator == (const TimeStamp & t) const
{
	return (seconds == t.seconds) && (usec == t.usec);
}

bool TimeStamp::operator != (const TimeStamp & t) const
{
	return !(*this == t);
}

bool TimeStamp::operator > (const TimeStamp & t) const
{
	if (seconds > t.seconds) return true;
	if (seconds == t.seconds && usec > t.usec) return true;
	return false;
}

bool TimeStamp::operator < (const TimeStamp & t) const
{
	if (seconds < t.seconds) return true;
	if (seconds == t.seconds && usec < t.usec) return true;
	return false;
}

TimeStamp & TimeStamp::operator = (const TimeStamp & t)
{
	seconds = t.seconds;
	usec    = t.usec;
	return *this;
}

TimeStamp & TimeStamp::operator = (const long & t)
{
	seconds = (size_t)(t / 1000000L);
	usec    = (size_t)(t % 1000000L);
	return *this;
}

TimeStamp & TimeStamp::operator += (const TimeStamp & m)
{
	usec += m.usec;
	seconds += m.seconds;
	if (usec >= 1000000) {
		usec -= 1000000;
		seconds++;
	}
	return *this;
}

TimeStamp & TimeStamp::operator -= (const TimeStamp & m)
{
	if (usec >= m.usec) {
		usec -= m.usec;
	} else {
		usec = 1000000 + usec - m.usec;
		seconds--;
	}
	seconds -= m.seconds;
	return *this;
}

TimeStamp & TimeStamp::operator += (const long & m)
{
	long total_usec = (long)usec + m;
	if (total_usec >= 0) {
		seconds += (size_t)(total_usec / 1000000L);
		usec = (size_t)(total_usec % 1000000L);
	} else {
		long abs_usec = -total_usec;
		size_t borrow_sec = (size_t)(abs_usec / 1000000L) + 1;
		seconds -= borrow_sec;
		usec = (size_t)(borrow_sec * 1000000L + total_usec);
	}
	return *this;
}

TimeStamp & TimeStamp::operator -= (const long & m)
{
	return (*this += (-m));
}

TimeStamp TimeStamp::operator + (const long & m) const
{
	TimeStamp result = *this;
	result += m;
	return result;
}

TimeStamp TimeStamp::operator - (const long & m) const
{
	TimeStamp result = *this;
	result -= m;
	return result;
}

long TimeStamp::operator + (const TimeStamp & a) const
{
	long result = (long)(seconds + a.seconds) * 1000000L;
	result += (long)(usec + a.usec);
	return result;
}

long TimeStamp::operator - (const TimeStamp & a) const
{
	long result = (long)(seconds - a.seconds) * 1000000L;
	result += (long)(usec - a.usec);
	return result;
}

double TimeStamp::diff(const TimeStamp & a) const
{
	double d = (double)(seconds - a.seconds);
	d += (double)((long)usec - (long)a.usec) / 1000000.0;
	return d;
}

double TimeStamp::time() const
{
	return (double)seconds + (double)usec / 1000000.0;
}

double TimeStamp::secOfDay() const
{
	return (double)(seconds % 86400) + (double)usec / 1000000.0;
}

size_t TimeStamp::get_seconds() const { return seconds; }
size_t TimeStamp::get_usec() const { return usec; }

/* ---- IntervalTimer ---- */

IntervalTimer::IntervalTimer(float rate, struct sigevent * sePtr)
	: run_rate(rate)
#ifdef __linux__
	, timer_id(0)
	, timer_created(false)
#endif
	, has_custom_event(false)
{
	memset(&sev, 0, sizeof(sev));

	if (sePtr != NULL) {
		sev = *sePtr;
		has_custom_event = true;
	} else {
		sev.sigev_notify = SIGEV_SIGNAL;
		sev.sigev_signo  = SIGALRM;
		sev.sigev_value.sival_int = 0;
	}
}

IntervalTimer::~IntervalTimer()
{
	stop();
}

void IntervalTimer::start()
{
	if (run_rate <= 0.0f) return;

#ifdef __linux__
	if (!timer_created) {
		if (timer_create(CLOCK_REALTIME, &sev, &timer_id) != 0)
			return;
		timer_created = true;
	}

	long period_ns = (long)(SEC2NANO / run_rate);
	struct itimerspec its;
	its.it_value.tv_sec    = period_ns / 1000000000L;
	its.it_value.tv_nsec   = period_ns % 1000000000L;
	its.it_interval.tv_sec  = its.it_value.tv_sec;
	its.it_interval.tv_nsec = its.it_value.tv_nsec;

	timer_settime(timer_id, 0, &its, NULL);
#else
	long period_us = (long)(SEC2MICRO / run_rate);
	struct itimerval itv;
	itv.it_value.tv_sec    = period_us / 1000000L;
	itv.it_value.tv_usec   = period_us % 1000000L;
	itv.it_interval.tv_sec  = itv.it_value.tv_sec;
	itv.it_interval.tv_usec = itv.it_value.tv_usec;

	setitimer(ITIMER_REAL, &itv, NULL);
#endif
}

void IntervalTimer::stop()
{
#ifdef __linux__
	if (timer_created) {
		struct itimerspec its;
		memset(&its, 0, sizeof(its));
		timer_settime(timer_id, 0, &its, NULL);
		timer_delete(timer_id);
		timer_created = false;
	}
#else
	struct itimerval itv;
	memset(&itv, 0, sizeof(itv));
	setitimer(ITIMER_REAL, &itv, NULL);
#endif
}

void IntervalTimer::setRunRate(float rate)
{
	run_rate = rate;
}

float IntervalTimer::getRunRate() const
{
	return run_rate;
}
