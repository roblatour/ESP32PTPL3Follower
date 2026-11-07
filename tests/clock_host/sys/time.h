#pragma once

#include "clock_host.h"

struct clock_host_timeval { time_t tv_sec; long tv_usec; };
int clock_host_gettime(int clock, timespec* value);
int clock_host_settime(int clock, const timespec* value);
int clock_host_settimeofday(const clock_host_timeval* value, const void* zone);
int clock_host_adjtime(const clock_host_timeval* value, clock_host_timeval* previous);

#ifdef CLOCK_REALTIME
#undef CLOCK_REALTIME
#endif
#define CLOCK_REALTIME host_realtime
#define clockid_t int
#define suseconds_t long
#define timeval clock_host_timeval
#define clock_gettime clock_host_gettime
#define clock_settime clock_host_settime
#define settimeofday clock_host_settimeofday
#define adjtime clock_host_adjtime
