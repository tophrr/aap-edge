#include "config_handler.h"
#include "config.h"
#include "fsm.h"
#include "network_mgr.h"
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
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
        Log.printf("[Config] JSON parse error: %s\n", error.c_str());
        return;
    }

    bool persist = false;
    if (doc["persist"].is<bool>()) {
        persist = doc["persist"].as<bool>();
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
                    Log.printf("[Config] Restarting OTA on port %d\n", g_config.ota_port);
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
                Log.println("[Config] Enabling OTA");
                ArduinoOTA.setPort(g_config.ota_port);
                ArduinoOTA.begin();
            } else if (!g_config.ota_enabled && otaWasEnabled) {
                Log.println("[Config] Disabling OTA");
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

        if (doc["mqtt_logs_enabled"].is<bool>()) {
            g_config.mqtt_logs_enabled = doc["mqtt_logs_enabled"].as<bool>();
        }

        if (doc["udp_stream_enabled"].is<bool>()) {
            g_config.udp_stream_enabled = doc["udp_stream_enabled"];
        }
        if (doc["udp_host"].is<const char*>()) {
            strncpy(g_config.udp_host, doc["udp_host"].as<const char*>(), sizeof(g_config.udp_host) - 1);
            g_config.udp_host[sizeof(g_config.udp_host) - 1] = '\0';
        }
        if (doc["udp_port"].is<int>()) {
            g_config.udp_port = doc["udp_port"];
        }

        fsm.applyConfig(g_config);

        if (persist) {
            saveConfigToNVS(g_config);
        }

        xSemaphoreGive(fsmMutex);
    }

    Log.println("[Config] Applied new configuration.");
    networkMgr.publishConfigAck(g_config);
}

void configRequestCallback() {
    if (fsmMutex != nullptr && xSemaphoreTake(fsmMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        networkMgr.publishConfigAck(g_config);
        xSemaphoreGive(fsmMutex);
    }
}

// ── NVS Persistence Helpers ──────────────────────────────────────────────────

void loadConfigFromNVS(RuntimeConfig& cfg) {
    Preferences prefs;
    // Open in read-only mode (true)
    if (!prefs.begin("rt_config", true)) {
        Log.println("[NVS] Namespace 'rt_config' not found. Using defaults.");
        return;
    }

    cfg.main_snr_threshold   = prefs.getFloat("main_snr", cfg.main_snr_threshold);
    cfg.sec_snr_threshold    = prefs.getFloat("sec_snr", cfg.sec_snr_threshold);
    cfg.confirm_sec          = prefs.getFloat("confirm_sec", cfg.confirm_sec);
    cfg.probing_timeout_sec  = prefs.getFloat("probing_timeout", cfg.probing_timeout_sec);
    cfg.active_timeout_sec   = prefs.getFloat("active_timeout", cfg.active_timeout_sec);
    cfg.pulse_on_sec         = prefs.getFloat("pulse_on_sec", cfg.pulse_on_sec);
    cfg.pulse_off_sec        = prefs.getFloat("pulse_off_sec", cfg.pulse_off_sec);
    cfg.pulse_tolerance_sec  = prefs.getFloat("pulse_tol", cfg.pulse_tolerance_sec);

    cfg.cycle_target_ms      = prefs.getUInt("cycle_target", cfg.cycle_target_ms);
    cfg.cycle_tolerance_ms   = prefs.getUInt("cycle_tol", cfg.cycle_tolerance_ms);
    cfg.required_cycles      = prefs.getInt("req_cycles", cfg.required_cycles);
    cfg.signal_streak_min    = prefs.getInt("sig_streak", cfg.signal_streak_min);

    cfg.main_freq_hz         = prefs.getFloat("main_freq", cfg.main_freq_hz);
    cfg.sec_freq_hz          = prefs.getFloat("sec_freq", cfg.sec_freq_hz);
    cfg.amb_freq_hz          = prefs.getFloat("amb_freq", cfg.amb_freq_hz);
    cfg.alpha_attack         = prefs.getFloat("alpha_attack", cfg.alpha_attack);
    cfg.alpha_decay          = prefs.getFloat("alpha_decay", cfg.alpha_decay);

    cfg.deep_sleep_enabled   = prefs.getBool("ds_enabled", cfg.deep_sleep_enabled);
    cfg.sleep_start_hour     = prefs.getInt("sleep_start", cfg.sleep_start_hour);
    cfg.wake_end_hour        = prefs.getInt("wake_end", cfg.wake_end_hour);

    cfg.led_enabled          = prefs.getBool("led_enabled", cfg.led_enabled);
    cfg.ota_enabled          = prefs.getBool("ota_enabled", cfg.ota_enabled);
    cfg.ota_port             = prefs.getInt("ota_port", cfg.ota_port);
    cfg.debug_enabled        = prefs.getBool("debug_enabled", cfg.debug_enabled);
    cfg.mqtt_logs_enabled    = prefs.getBool("mqtt_logs", cfg.mqtt_logs_enabled);

    cfg.udp_stream_enabled   = prefs.getBool("udp_enabled", cfg.udp_stream_enabled);
    prefs.getString("udp_host", cfg.udp_host, sizeof(cfg.udp_host));
    cfg.udp_port             = prefs.getUInt("udp_port", cfg.udp_port);

    prefs.end();
    Log.println("[NVS] Configuration successfully loaded from NVS.");
}

void saveConfigToNVS(const RuntimeConfig& cfg) {
    Preferences prefs;
    // Open in read-write mode (false)
    if (!prefs.begin("rt_config", false)) {
        Log.println("[NVS] Error: Failed to open namespace 'rt_config' for writing.");
        return;
    }

    prefs.putFloat("main_snr", cfg.main_snr_threshold);
    prefs.putFloat("sec_snr", cfg.sec_snr_threshold);
    prefs.putFloat("confirm_sec", cfg.confirm_sec);
    prefs.putFloat("probing_timeout", cfg.probing_timeout_sec);
    prefs.putFloat("active_timeout", cfg.active_timeout_sec);
    prefs.putFloat("pulse_on_sec", cfg.pulse_on_sec);
    prefs.putFloat("pulse_off_sec", cfg.pulse_off_sec);
    prefs.putFloat("pulse_tol", cfg.pulse_tolerance_sec);

    prefs.putUInt("cycle_target", cfg.cycle_target_ms);
    prefs.putUInt("cycle_tol", cfg.cycle_tolerance_ms);
    prefs.putInt("req_cycles", cfg.required_cycles);
    prefs.putInt("sig_streak", cfg.signal_streak_min);

    prefs.putFloat("main_freq", cfg.main_freq_hz);
    prefs.putFloat("sec_freq", cfg.sec_freq_hz);
    prefs.putFloat("amb_freq", cfg.amb_freq_hz);
    prefs.putFloat("alpha_attack", cfg.alpha_attack);
    prefs.putFloat("alpha_decay", cfg.alpha_decay);

    prefs.putBool("ds_enabled", cfg.deep_sleep_enabled);
    prefs.putInt("sleep_start", cfg.sleep_start_hour);
    prefs.putInt("wake_end", cfg.wake_end_hour);

    prefs.putBool("led_enabled", cfg.led_enabled);
    prefs.putBool("ota_enabled", cfg.ota_enabled);
    prefs.putInt("ota_port", cfg.ota_port);
    prefs.putBool("debug_enabled", cfg.debug_enabled);
    prefs.putBool("mqtt_logs", cfg.mqtt_logs_enabled);

    prefs.putBool("udp_enabled", cfg.udp_stream_enabled);
    prefs.putString("udp_host", cfg.udp_host);
    prefs.putUInt("udp_port", cfg.udp_port);

    prefs.end();
    Log.println("[NVS] Configuration successfully saved to NVS.");
}
