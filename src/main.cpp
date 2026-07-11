#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <ArduinoJson.h>

#include "config.h"
#include "fsm.h"
#include "network_mgr.h"

// ── Forward declarations ────────────────────────────────────────────────────

extern void initI2S();
extern void audioDspTask(void* parameter);

static void fsmTask(void* parameter);
static void configCallback(const char* json);
static void checkDeepSleep(time_t now);
static void syncNTP();

// ── Globals ─────────────────────────────────────────────────────────────────

static QueueHandle_t audioQueue = nullptr;
static NetworkMgr networkMgr;
static FSMEngine fsm;
static SemaphoreHandle_t fsmMutex = nullptr;

bool g_debugEnabled = false;

// ── Setup ───────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(921600);
    delay(500);
    Serial.println("\n\n===================================");
    Serial.println("Acoustic ATCS Proxy Node");
    Serial.println("===================================");

    // Initialize I2S microphone
    initI2S();

    // Start Wi-Fi + MQTT (async)
    networkMgr.begin();
    networkMgr.onConfig(configCallback);

    // NTP sync will complete during first few seconds of runtime
    syncNTP();

    // Create audio queue (IPC between Core 0 and Core 1)
    audioQueue = xQueueCreate(AUDIO_QUEUE_SIZE, sizeof(AudioFrame));
    if (audioQueue == nullptr) {
        Serial.println("[FATAL] Failed to create audio queue");
        abort();
    }

    fsmMutex = xSemaphoreCreateMutex();

    // Create tasks pinned to cores
    // Core 0 (PRO_CPU) — deterministic audio timing
    xTaskCreatePinnedToCore(
        audioDspTask,       // task function
        "audioDsp",         // name
        4096 + (NUM_SAMPLES * 2 * sizeof(float)) + 1024,
        (void*)audioQueue,  // parameters
        2,                  // priority
        nullptr,            // task handle
        0                   // core (PRO_CPU)
    );

    // Core 1 (APP_CPU) — FSM logic + networking
    xTaskCreatePinnedToCore(
        fsmTask,
        "fsm",
        4096 + (NUM_SAMPLES * sizeof(float)) + 1024,
        (void*)audioQueue,
        1,
        nullptr,
        1  // core (APP_CPU)
    );

    Serial.println("[Main] Tasks created. Starting...");
}

// ── Loop (unused — tasks handle everything) ─────────────────────────────────

void loop() {
    delay(1000);  // Deep sleep check happens in fsmTask
}

// ── FSM Task (Core 1) ───────────────────────────────────────────────────────

static void fsmTask(void* parameter) {
    QueueHandle_t queue = (QueueHandle_t)parameter;
    AudioFrame frame;
    unsigned long uptimeSec = 0;

    Serial.println("[FSM] Task started on Core 1");

    while (true) {
        // Block until audio frame arrives
        if (xQueueReceive(queue, &frame, portMAX_DELAY) == pdPASS) {
            time_t now = time(nullptr);
            uptimeSec = millis() / 1000;

            // Maintain network connection
            networkMgr.loop();

            // Process frame through FSM (protected by mutex for config safety)
            FSMEvent event;
            if (xSemaphoreTake(fsmMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                event = fsm.processFrame(frame, (int64_t)now);
                xSemaphoreGive(fsmMutex);
            }

            // Publish event if state transition occurred
            if (event.type == FSMEvent::START) {
                Serial.printf("[Event] START @ %lld (probing=%lld, active=%lld)\n",
                    (long long)event.timestamp_sec,
                    (long long)event.probing_started_sec,
                    (long long)event.active_at_sec);
                networkMgr.publishEvent("started", event.timestamp_sec,
                    event.probing_started_sec, event.active_at_sec);
            } else if (event.type == FSMEvent::END) {
                Serial.printf("[Event] END @ %lld (duration=%.1f)\n",
                    (long long)event.timestamp_sec, (double)event.duration_sec);
                networkMgr.publishEvent("ended", event.timestamp_sec,
                    event.probing_started_sec, event.active_at_sec,
                    event.duration_sec);
            }

            // ── Periodic status line ────────────────────────────────────
            if (g_debugEnabled) {
                static unsigned long lastStatusPrintMs = 0;
                unsigned long nowMs = millis();
                if (nowMs - lastStatusPrintMs >= DEBUG_PRINT_INTERVAL_MS) {
                    lastStatusPrintMs = nowMs;
                    int st;
                    if (xSemaphoreTake(fsmMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        st = fsm.state();
                        xSemaphoreGive(fsmMutex);
                    } else {
                        st = -1;
                    }
                    static const char* stateNames[] = {"IDLE", "PROBING", "ACTIVE"};
                    const char* stName = (st >= 0 && st <= 2) ? stateNames[st] : "???";
                    Serial.printf("[STATUS] FSM=%s | main=%.1f sec=%.1f amb=%.1f | heap=%u rssi=%d\n",
                        stName,
                        (double)frame.main_db, (double)frame.sec_db, (double)frame.amb_db,
                        ESP.getFreeHeap(), WiFi.RSSI());
                }
            }

            // Periodic heartbeat
            if (networkMgr.timeSynced() &&
                uptimeSec - networkMgr.lastHeartbeatSec() >= HEARTBEAT_INTERVAL_SEC) {
                networkMgr.publishHeartbeat(uptimeSec);
            }

            // Periodic telemetry
            if (networkMgr.timeSynced() &&
                uptimeSec - networkMgr.lastTelemetrySec() >= TELEMETRY_INTERVAL_SEC) {
                int state;
                int eventsToday;
                if (xSemaphoreTake(fsmMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    state = fsm.state();
                    eventsToday = fsm.eventsToday();
                    xSemaphoreGive(fsmMutex);
                } else {
                    state = -1;
                    eventsToday = -1;
                }
                networkMgr.publishTelemetry(
                    WiFi.RSSI(), WiFi.RSSI(),
                    ESP.getFreeHeap(), ESP.getMinFreeHeap(),
                    uptimeSec, state, eventsToday,
                    networkMgr.mqttReconnects()
                );
            }

            // Check deep sleep
            if (now > 100000 && DEEP_SLEEP_ENABLED) {
                checkDeepSleep(now);
            }
        }
    }
}

// ── Config Callback ─────────────────────────────────────────────────────────

static void configCallback(const char* json) {
    // Parse JSON using ArduinoJson v7
    // Only override fields that are present in the message
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json);
    if (error) {
        Serial.printf("[Config] JSON parse error: %s\n", error.c_str());
        return;
    }

    RuntimeConfig cfg = fsm.getConfig();

    if (doc["main_snr_db"].is<float>())          cfg.main_snr_threshold = doc["main_snr_db"];
    if (doc["sec_snr_db"].is<float>())           cfg.sec_snr_threshold = doc["sec_snr_db"];
    if (doc["confirm_sec"].is<float>())          cfg.confirm_sec = doc["confirm_sec"];
    if (doc["probing_timeout_sec"].is<float>())  cfg.probing_timeout_sec = doc["probing_timeout_sec"];
    if (doc["active_timeout_sec"].is<float>())   cfg.active_timeout_sec = doc["active_timeout_sec"];
    if (doc["debug_enabled"].is<bool>()) {
        g_debugEnabled = doc["debug_enabled"].as<bool>();
    } else if (doc["debug"].is<bool>()) {
        g_debugEnabled = doc["debug"].as<bool>();
    }
    cfg.debug_enabled = g_debugEnabled;

    if (fsmMutex != nullptr && xSemaphoreTake(fsmMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        fsm.applyConfig(cfg);
        xSemaphoreGive(fsmMutex);
    }

    Serial.println("[Config] Applied new configuration:");
    Serial.printf("  main_snr_db=%.1f, sec_snr_db=%.1f\n",
        (double)cfg.main_snr_threshold, (double)cfg.sec_snr_threshold);
    Serial.printf("  confirm_sec=%.1f, probing_timeout=%.1f, active_timeout=%.1f\n",
        (double)cfg.confirm_sec, (double)cfg.probing_timeout_sec, (double)cfg.active_timeout_sec);
    Serial.printf("  debug_enabled=%s\n", g_debugEnabled ? "true" : "false");
}

// ── NTP Sync ───────────────────────────────────────────────────────────────

static void syncNTP() {
    configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    Serial.println("[NTP] Syncing...");
    // NTP sync happens asynchronously; check timeSynced() before using timestamps
}

// ── Deep Sleep ──────────────────────────────────────────────────────────────

static void checkDeepSleep(time_t now) {
    if (!DEEP_SLEEP_ENABLED) return;

    struct tm* timeinfo = localtime(&now);
    int hour = timeinfo->tm_hour;

    // Outside operating hours (05:00–22:00)
    if (hour >= SLEEP_START_HOUR || hour < WAKE_END_HOUR) {
        Serial.printf("[Sleep] Hour %d — entering deep sleep\n", hour);

        // Calculate seconds until next wake time (05:00)
        time_t wakeTime = now;
        struct tm wakeTm = *timeinfo;
        wakeTm.tm_hour = WAKE_END_HOUR;
        wakeTm.tm_min = 0;
        wakeTm.tm_sec = 0;

        wakeTime = mktime(&wakeTm);
        if (hour >= SLEEP_START_HOUR) {
            // Wake up tomorrow
            wakeTime += 86400;
        }

        uint64_t sleepUs = (uint64_t)(difftime(wakeTime, now) * 1000000ULL);

        Serial.printf("[Sleep] Sleeping for %llu seconds\n", (unsigned long long)(sleepUs / 1000000));

        esp_sleep_enable_timer_wakeup(sleepUs);
        esp_deep_sleep_start();
    }
} 
