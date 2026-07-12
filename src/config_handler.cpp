#include "config_handler.h"
#include "config.h"
#include "fsm.h"
#include "network_mgr.h"
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ── Globals owned by main.cpp ────────────────────────────────────────────────

extern FSMEngine          fsm;
extern SemaphoreHandle_t  fsmMutex;
extern NetworkMgr         networkMgr;

// ── Config Callback ──────────────────────────────────────────────────────────

void configCallback(const char* json) {
    // Parse JSON using ArduinoJson v7
    // Only override fields that are present in the message
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json);
    if (error) {
        Serial.printf("[Config] JSON parse error: %s\n", error.c_str());
        return;
    }

    if (fsmMutex != nullptr && xSemaphoreTake(fsmMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // FSM Timing Parameters
        if (doc["main_snr_db"].is<float>())          g_config.main_snr_threshold = doc["main_snr_db"];
        if (doc["sec_snr_db"].is<float>())           g_config.sec_snr_threshold = doc["sec_snr_db"];
        if (doc["confirm_sec"].is<float>())          g_config.confirm_sec = doc["confirm_sec"];
        if (doc["probing_timeout_sec"].is<float>())  g_config.probing_timeout_sec = doc["probing_timeout_sec"];
        if (doc["active_timeout_sec"].is<float>())   g_config.active_timeout_sec = doc["active_timeout_sec"];
        if (doc["pulse_on_sec"].is<float>())         g_config.pulse_on_sec = doc["pulse_on_sec"];
        if (doc["pulse_off_sec"].is<float>())        g_config.pulse_off_sec = doc["pulse_off_sec"];
        if (doc["pulse_tolerance_sec"].is<float>())  g_config.pulse_tolerance_sec = doc["pulse_tolerance_sec"];

        // Pulse validation parameters
        if (doc["cycle_target_ms"].is<unsigned long>()) g_config.cycle_target_ms = doc["cycle_target_ms"];
        if (doc["cycle_tolerance_ms"].is<unsigned long>()) g_config.cycle_tolerance_ms = doc["cycle_tolerance_ms"];
        if (doc["required_cycles"].is<int>())        g_config.required_cycles = doc["required_cycles"];
        if (doc["signal_streak_min"].is<int>())      g_config.signal_streak_min = doc["signal_streak_min"];

        // DSP Parameters
        if (doc["main_freq_hz"].is<float>())         g_config.main_freq_hz = doc["main_freq_hz"];
        if (doc["sec_freq_hz"].is<float>())          g_config.sec_freq_hz = doc["sec_freq_hz"];
        if (doc["amb_freq_hz"].is<float>())          g_config.amb_freq_hz = doc["amb_freq_hz"];
        if (doc["alpha_attack"].is<float>())         g_config.alpha_attack = doc["alpha_attack"];
        if (doc["alpha_decay"].is<float>())          g_config.alpha_decay = doc["alpha_decay"];

        // Deep Sleep Parameters
        if (doc["deep_sleep_enabled"].is<bool>())    g_config.deep_sleep_enabled = doc["deep_sleep_enabled"];
        if (doc["sleep_start_hour"].is<int>())       g_config.sleep_start_hour = doc["sleep_start_hour"];
        if (doc["wake_end_hour"].is<int>())          g_config.wake_end_hour = doc["wake_end_hour"];

        // LED status
        if (doc["led_enabled"].is<bool>())           g_config.led_enabled = doc["led_enabled"];

        // OTA Parameters
        if (doc["ota_port"].is<int>()) {
            int newPort = doc["ota_port"];
            if (newPort != g_config.ota_port) {
                g_config.ota_port = newPort;
                if (g_config.ota_enabled) {
                    Serial.printf("[Config] Restarting OTA on port %d\n", g_config.ota_port);
                    ArduinoOTA.end();
                    ArduinoOTA.setPort(g_config.ota_port);
                    ArduinoOTA.begin();
                }
            }
        }
        if (doc["ota_enabled"].is<bool>()) {
            bool otaWasEnabled = g_config.ota_enabled;
            g_config.ota_enabled = doc["ota_enabled"];
            if (g_config.ota_enabled && !otaWasEnabled) {
                Serial.println("[Config] Enabling OTA");
                ArduinoOTA.setPort(g_config.ota_port);
                ArduinoOTA.begin();
            } else if (!g_config.ota_enabled && otaWasEnabled) {
                Serial.println("[Config] Disabling OTA");
                ArduinoOTA.end();
            }
        }

        // Debug
        if (doc["debug_enabled"].is<bool>()) {
            g_debugEnabled = doc["debug_enabled"].as<bool>();
        } else if (doc["debug"].is<bool>()) {
            g_debugEnabled = doc["debug"].as<bool>();
        }
        g_config.debug_enabled = g_debugEnabled;

        fsm.applyConfig(g_config);
        xSemaphoreGive(fsmMutex);
    }

    Serial.println("[Config] Applied new configuration.");
    networkMgr.publishConfigAck(g_config);
}

void configRequestCallback() {
    if (fsmMutex != nullptr && xSemaphoreTake(fsmMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        networkMgr.publishConfigAck(g_config);
        xSemaphoreGive(fsmMutex);
    }
}
