#pragma once

#include <Arduino.h>
#include "mqtt_logger.h"

extern bool g_debugEnabled;

#define DEBUG_PRINT(...) do { if (g_debugEnabled) Log.print(__VA_ARGS__); } while (0)
#define DEBUG_PRINTLN(...) do { if (g_debugEnabled) Log.println(__VA_ARGS__); } while (0)
#define DEBUG_PRINTF(...) do { if (g_debugEnabled) Log.printf(__VA_ARGS__); } while (0)
#define DEBUG_PRINT_INTERVAL_MS 1000u

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
#define MQTT_TOPIC_CONFIG_ACK       "crossing/config/ack"
#define MQTT_TOPIC_CONFIG_REQ       "crossing/config/req"
#define MQTT_TOPIC_RTT              "crossing/rtt"
#define MQTT_TOPIC_LOG              "crossing/log"

#define HEARTBEAT_INTERVAL_SEC      60
#define TELEMETRY_INTERVAL_SEC      300

// ==========================================
// HARDWARE & PIN MAPPING (ESP32 DevKit V1)
// ==========================================
#define I2S_PORT        I2S_NUM_0
#define I2S_WS_PIN      21
#define I2S_SCK_PIN     19
#define I2S_SD_PIN      18

#define STATUS_LED_PIN  2       // GPIO2 — built-in LED: off=idle, blink=probing, on=active
#define LED_BLINK_MS    250     // Blink interval for PROBING state

// ==========================================
// DSP & AUDIO PARAMETERS
// ==========================================
#define SAMPLE_RATE     16000
#define WINDOW_SEC      0.1f
#define NUM_SAMPLES     (int)(SAMPLE_RATE * WINDOW_SEC)  // 1600

#define MAIN_FREQ_HZ    1253.0f
#define SEC_FREQ_HZ     662.0f
#define AMB_FREQ_HZ     900.0f

#define MAIN_SNR_DB     25.0f
#define SEC_SNR_DB      22.0f

#define ALPHA_ATTACK    0.01f
#define ALPHA_DECAY     0.20f

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
// OTA (Over-the-Air) Update
// ==========================================
#define OTA_ENABLED             1
#define OTA_PORT                3232

// ==========================================
// FREERTOS
// ==========================================
#define AUDIO_QUEUE_SIZE    10
#define UDP_CHUNK_SIZE      400
#define UDP_QUEUE_SIZE      8

struct AudioUdpPacket {
    int16_t samples[UDP_CHUNK_SIZE];
};

// ==========================================
// RUNTIME CONFIGURATION & STATE
// ==========================================
enum FSMState : uint8_t {
    FSM_IDLE = 0,
    FSM_PROBING = 1,
    FSM_ACTIVE = 2
};

struct RuntimeConfig {
    // FSM Timing Parameters
    float main_snr_threshold = MAIN_SNR_DB;
    float sec_snr_threshold = SEC_SNR_DB;
    float confirm_sec = CONFIRM_SEC;
    float probing_timeout_sec = PROBING_TIMEOUT_SEC;
    float active_timeout_sec = ACTIVE_TIMEOUT_SEC;
    float pulse_on_sec = PULSE_ON_SEC;
    float pulse_off_sec = PULSE_OFF_SEC;
    float pulse_tolerance_sec = PULSE_TOLERANCE_SEC;

    // Pulse validation parameters
    unsigned long cycle_target_ms = 1200;
    unsigned long cycle_tolerance_ms = 300;
    int required_cycles = 2;
    int signal_streak_min = 2;

    // DSP Parameters
    float main_freq_hz = MAIN_FREQ_HZ;
    float sec_freq_hz = SEC_FREQ_HZ;
    float amb_freq_hz = AMB_FREQ_HZ;
    float alpha_attack = ALPHA_ATTACK;
    float alpha_decay = ALPHA_DECAY;

    // Deep Sleep Parameters
    bool deep_sleep_enabled = DEEP_SLEEP_ENABLED;
    int sleep_start_hour = SLEEP_START_HOUR;
    int wake_end_hour = WAKE_END_HOUR;

    // LED Toggle / Status
    bool led_enabled = true;

    // OTA Parameters
    bool ota_enabled = OTA_ENABLED;
    int ota_port = OTA_PORT;

    // UDP Audio Streaming
    bool udp_stream_enabled = false;
    char udp_host[64] = "";
    uint16_t udp_port = 5001;

    // Debug
    bool debug_enabled = false;
    bool mqtt_logs_enabled = false;
};

struct PulseValidationState {
    unsigned long lastRisingEdgeMs = 0;
    int validCycleCount = 0;
    unsigned long lastSignalMs = 0;
    int signalStreak = 0;
};

extern RuntimeConfig g_config;