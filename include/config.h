#pragma once

#include <Arduino.h>

extern bool g_debugEnabled;

#define DEBUG_PRINT(...) do { if (g_debugEnabled) Serial.print(__VA_ARGS__); } while (0)
#define DEBUG_PRINTLN(...) do { if (g_debugEnabled) Serial.println(__VA_ARGS__); } while (0)
#define DEBUG_PRINTF(...) do { if (g_debugEnabled) Serial.printf(__VA_ARGS__); } while (0)

// Load credentials from secrets.h (gitignored — see secrets.h.example)
// If missing, provides placeholder defaults so the build doesn't break.
#if __has_include("secrets.h")
    #include "secrets.h"
#else
    #pragma message "------------------------------------------------------------"
    #pragma message "secrets.h not found! Copy include/secrets.h.example to"
    #pragma message "include/secrets.h and fill in your real credentials."
    #pragma message "------------------------------------------------------------"
    #define WIFI_SSID           "CHANGE_ME"
    #define WIFI_PASSWORD       ""
    #define WIFI_EAP_IDENTITY   ""
    #define WIFI_EAP_USERNAME   ""
    #define WIFI_EAP_PASSWORD   ""
    #define MQTT_SERVER         "CHANGE_ME"
    #define MQTT_PORT           1883
#endif

// ==========================================
// MQTT TOPICS
// ==========================================
#define MQTT_TOPIC_EVENT            "crossing/event"
#define MQTT_TOPIC_HEARTBEAT        "crossing/heartbeat"
#define MQTT_TOPIC_TELEMETRY        "crossing/telemetry"
#define MQTT_TOPIC_CONFIG           "crossing/config"

#define HEARTBEAT_INTERVAL_SEC      60
#define TELEMETRY_INTERVAL_SEC      300

// ==========================================
// HARDWARE & PIN MAPPING (ESP32 DevKit V1)
// ==========================================
#define I2S_PORT        I2S_NUM_0
#define I2S_WS_PIN      21
#define I2S_SCK_PIN     19
#define I2S_SD_PIN      18

// ==========================================
// DSP & AUDIO PARAMETERS
// ==========================================
#define SAMPLE_RATE     16000
#define WINDOW_SEC      0.1f
#define NUM_SAMPLES     (int)(SAMPLE_RATE * WINDOW_SEC)  // 1600

#define MAIN_FREQ_HZ    1253.0f
#define SEC_FREQ_HZ     662.0f
#define AMB_FREQ_HZ     900.0f

#define MAIN_SNR_DB     8.0f
#define SEC_SNR_DB      4.0f

// ==========================================
// FSM TIMING PARAMETERS (Seconds)
// ==========================================
#define CONFIRM_SEC           0.7f
#define PROBING_TIMEOUT_SEC   3.0f
#define ACTIVE_TIMEOUT_SEC    2.5f
#define PULSE_ON_SEC          0.60f
#define PULSE_OFF_SEC         0.60f
#define PULSE_TOLERANCE_SEC   0.20f

// ==========================================
// DEEP SLEEP SCHEDULE
// ==========================================
#define DEEP_SLEEP_ENABLED      0       // Set to 0 to disable deep sleep
#define SLEEP_START_HOUR    22
#define WAKE_END_HOUR       5

// ==========================================
// FREERTOS
// ==========================================
#define AUDIO_QUEUE_SIZE    10