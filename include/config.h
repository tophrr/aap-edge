#pragma once

// ==========================================
// NETWORK CONFIGURATION
// ==========================================
#define WIFI_SSID           "YOUR_WIFI_SSID"
#define WIFI_PASSWORD       "YOUR_WIFI_PASSWORD"
#define WIFI_EAP_IDENTITY   "anonymous@domain"
#define WIFI_EAP_USERNAME   ""
#define WIFI_EAP_PASSWORD   ""

#define MQTT_SERVER                 "192.168.1.100"
#define MQTT_PORT                   1883
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
#define I2S_WS_PIN      15
#define I2S_SCK_PIN     14
#define I2S_SD_PIN      32

// ==========================================
// DSP & AUDIO PARAMETERS
// ==========================================
#define SAMPLE_RATE     8000
#define WINDOW_SEC      0.1f
#define NUM_SAMPLES     (int)(SAMPLE_RATE * WINDOW_SEC)  // 800

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
#define SLEEP_START_HOUR    22
#define WAKE_END_HOUR       5

// ==========================================
// FREERTOS
// ==========================================
#define AUDIO_QUEUE_SIZE    10