#include "sleep.h"
#include "config.h"
#include <Arduino.h>
#include <esp_sleep.h>

// ── NTP Sync ─────────────────────────────────────────────────────────────────

void syncNTP() {
    configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    Serial.println("[NTP] Syncing...");
    // NTP sync happens asynchronously; check timeSynced() before using timestamps
}

// ── Deep Sleep ────────────────────────────────────────────────────────────────

void checkDeepSleep(time_t now) {
    if (!g_config.deep_sleep_enabled) return;

    struct tm* timeinfo = localtime(&now);
    int hour = timeinfo->tm_hour;

    // Outside operating hours
    if (hour >= g_config.sleep_start_hour || hour < g_config.wake_end_hour) {
        Serial.printf("[Sleep] Hour %d — entering deep sleep\n", hour);

        // Calculate seconds until next wake time
        struct tm wakeTm = *timeinfo;
        wakeTm.tm_hour = g_config.wake_end_hour;
        wakeTm.tm_min  = 0;
        wakeTm.tm_sec  = 0;

        time_t wakeTime = mktime(&wakeTm);
        if (hour >= g_config.sleep_start_hour) {
            // Wake up tomorrow
            wakeTime += 86400;
        }

        uint64_t sleepUs = (uint64_t)(difftime(wakeTime, now) * 1000000ULL);

        Serial.printf("[Sleep] Sleeping for %llu seconds\n",
                      (unsigned long long)(sleepUs / 1000000));

        esp_sleep_enable_timer_wakeup(sleepUs);
        esp_deep_sleep_start();
    }
}
