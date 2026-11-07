#pragma once

struct clock_host_timex { int modes; long freq; };
inline constexpr int ADJ_FREQUENCY = 1;
int clock_host_adjclock(int clock, clock_host_timex* value);
#define timex clock_host_timex
#define clock_adjtime clock_host_adjclock
