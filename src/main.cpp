#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <ArduinoOTA.h>

#include "config.h"
#include "fsm.h"
#include "network_mgr.h"
#include "config_handler.h"
#include "ota.h"
#include "sleep.h"

// ── Forward declarations ────────────────────────────────────────────────────

extern void initI2S();
extern void audioDspTask(void* parameter);

static void fsmTask(void* parameter);

// ── Globals ─────────────────────────────────────────────────────────────────

static QueueHandle_t audioQueue = nullptr;

// Non-static so config_handler.cpp can reference them via extern
NetworkMgr       networkMgr;
FSMEngine        fsm;
SemaphoreHandle_t fsmMutex = nullptr;

// Task handles exposed to NetworkMgr for FreeRTOS stack watermark telemetry
static TaskHandle_t audioDspHandle = nullptr;
static TaskHandle_t fsmTaskHandle  = nullptr;

bool g_debugEnabled = false;
RuntimeConfig g_config;

// ── LED State Indicator ─────────────────────────────────────────────────────

/// Update the status LED based on FSM state.
/// off=IDLE, blinking=PROBING, solid on=ACTIVE.
static void updateStatusLED(int currentState) {
    static int lastState = -1;
    static unsigned long lastToggleMs = 0;

    if (!g_config.led_enabled) {
        digitalWrite(STATUS_LED_PIN, LOW);
        lastState = -1;
        return;
    }

    if (currentState == lastState && currentState != FSM_PROBING) {
        return;  // No change needed for stable non-blinking states
    }
    lastState = currentState;

    switch (currentState) {
        case FSM_IDLE:
            digitalWrite(STATUS_LED_PIN, LOW);
            break;
        case FSM_PROBING: {
            unsigned long nowMs = millis();
            if (nowMs - lastToggleMs >= LED_BLINK_MS) {
                lastToggleMs = nowMs;
                digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
            }
            break;
        }
        case FSM_ACTIVE:
            digitalWrite(STATUS_LED_PIN, HIGH);
            break;
        default:
            break;
    }
}

// ── Setup ───────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(460800);
    delay(500);
    Serial.println("\n\n===================================");
    Serial.println("Acoustic ATCS Proxy Node");
    Serial.println("===================================");

    // Initialize status LED
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    // Initialize I2S microphone
    initI2S();

    // Start Wi-Fi + MQTT (async)
    networkMgr.begin();
    networkMgr.onConfig(configCallback);
    networkMgr.onConfigRequest(configRequestCallback);

    // NTP sync will complete during first few seconds of runtime
    syncNTP();

    // Initialize OTA (starts listening when WiFi is available)
    initOTA();

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
        &audioDspHandle,    // task handle (for stack watermark telemetry)
        0                   // core (PRO_CPU)
    );

    // Core 1 (APP_CPU) — FSM logic + networking
    xTaskCreatePinnedToCore(
        fsmTask,
        "fsm",
        4096 + (NUM_SAMPLES * sizeof(float)) + 1024,
        (void*)audioQueue,
        1,
        &fsmTaskHandle,     // task handle (for stack watermark telemetry)
        1  // core (APP_CPU)
    );

    // Expose handles to NetworkMgr so it can report stack HWMs in telemetry
    networkMgr.taskHandleFsm   = fsmTaskHandle;
    networkMgr.taskHandleAudio = audioDspHandle;

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

            // Handle OTA updates
            if (g_config.ota_enabled) {
                ArduinoOTA.handle();
            }

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

            // Update status LED based on FSM state
            {
                int st;
                if (xSemaphoreTake(fsmMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    st = fsm.state();
                    xSemaphoreGive(fsmMutex);
                } else {
                    st = -1;
                }
                updateStatusLED(st);
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
                // NetworkMgr queries all system metrics internally
                networkMgr.publishTelemetry(uptimeSec, state, eventsToday);
            }

            // Check deep sleep
            if (now > 100000 && g_config.deep_sleep_enabled) {
                checkDeepSleep(now);
            }
        }
    }
}
