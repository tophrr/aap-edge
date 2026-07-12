#pragma once

#include <time.h>

/// Kick off asynchronous NTP sync. Call once in setup().
void syncNTP();

/// Enter deep sleep if the current hour is outside the operating window.
/// No-op when deep_sleep_enabled is false.
void checkDeepSleep(time_t now);
