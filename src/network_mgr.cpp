#include "network_mgr.h"
#include "config.h"
#include "mqtt_logger.h"
#include <esp_wpa2.h>
#include <esp_wifi.h>
#include <esp_system.h>       // esp_reset_reason()
#include <esp_chip_info.h>
#include <ArduinoJson.h>

// ── Internal temperature sensor ──
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();  // Returns raw fahr value; convert via (val - 32) / 1.8
#ifdef __cplusplus
}
#endif

// ── Constructor ─────────────────────────────────────────────────────────────

NetworkMgr* NetworkMgr::_instance = nullptr;

NetworkMgr::NetworkMgr()
    : _mqttClient(_wifiClient)
    , _wifiConnecting(false)
    , _wifiReconnectMs(0)
    , _wifiWasConnected(false)
    , _wifiScanInProgress(false)
    , _wifiScanStartMs(0)
    , _wifiRoamingScanInProgress(false)
    , _lastRssiCheckMs(0)
    , _lastScanMs(0)
    , _wifiDisconnects(0)
    , _wifiFaults(0)
    , _mqttConnecting(false)
    , _mqttReconnectMs(0)
    , _mqttBackoffMs(1000)
    , _mqttReconnects(0)
    , _rttPending(false)
    , _rttSentMs(0)
    , _rttLastMs(-1)
    , _rttNextPingMs(0)
    , _lastHeartbeatSec(0)
    , _lastTelemetrySec(0)
    , _configCb(nullptr)
    , _configRequestCb(nullptr)
    , _telemetryRequestCb(nullptr)
{
    _mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    _mqttClient.setBufferSize(1024);
    _instance = this;
}

// ── Static MQTT Callback Dispatcher ─────────────────────────────────────────

void NetworkMgr::staticMqttCallback(char* topic, byte* payload, unsigned int length) {
    if (_instance) {
        _instance->_mqttCallback(topic, payload, length);
    }
}

// ── Public API ──────────────────────────────────────────────────────────────

void NetworkMgr::begin() {
    Log.println("[Network] Starting Wi-Fi...");
    WiFi.mode(WIFI_STA);
    
    // Set country code to ID (Indonesia) to support channels 12 & 13
    wifi_country_t country;
    memcpy(country.cc, "ID", 3);
    country.schan = 1;
    country.nchan = 13;
    country.policy = WIFI_COUNTRY_POLICY_AUTO;
    esp_wifi_set_country(&country);

    _connectWiFi();
}

void NetworkMgr::loop() {
    // Handle MQTT client loop
    if (_mqttClient.connected()) {
        _mqttClient.loop();

        // Dequeue and publish logs to MQTT if enabled
        if (g_config.mqtt_logs_enabled && g_logQueue != nullptr) {
            LogMessage msg;
            int sentCount = 0;
            // Limit to 5 logs per loop invocation to prevent flooding/blocking
            while (sentCount < 5 && xQueueReceive(g_logQueue, &msg, 0) == pdPASS) {
                _mqttClient.publish(MQTT_TOPIC_LOG, msg.text);
                sentCount++;
            }
        }
    }

    // ── Wi-Fi disconnect detection ──────────────────────────────────────
    bool wifiNow = (WiFi.status() == WL_CONNECTED);
    if (_wifiWasConnected && !wifiNow) {
        _wifiDisconnects++;
        Log.printf("[WiFi] Disconnected (total drops: %d)\n", _wifiDisconnects);
    }
    _wifiWasConnected = wifiNow;

    // ── Wi-Fi scanning progress ────────────────────────────────────────
    if (_wifiScanInProgress) {
        int16_t scanResult = WiFi.scanComplete();
        if (scanResult >= 0) {
            Log.printf("[WiFi] Connection scan complete. Found %d networks.\n", scanResult);
            _handleWiFiScanResults(false);
        } else if (scanResult == WIFI_SCAN_FAILED || (millis() - _wifiScanStartMs > 10000)) {
            Log.println("[WiFi] Connection scan failed or timed out. Falling back to standard connection.");
            WiFi.scanDelete();
            _wifiScanInProgress = false;
            // Fallback to standard connection
            _wifiConnecting = true;
            _wifiReconnectMs = millis();
            if (strlen(WIFI_EAP_USERNAME) > 0) {
                Log.println("[WiFi] Connecting to default SSID (WPA2-Enterprise)");
                WiFi.begin(WIFI_SSID, WIFI_EAP_METHOD, WIFI_EAP_IDENTITY, WIFI_EAP_USERNAME, WIFI_EAP_PASSWORD);
            } else {
                Log.println("[WiFi] Connecting to default SSID (WPA2-PSK)");
                esp_wifi_sta_wpa2_ent_disable();
                WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            }
        }
    }

    // ── Wi-Fi active roaming scan progress ────────────────────────────────
    if (_wifiRoamingScanInProgress) {
        int16_t scanResult = WiFi.scanComplete();
        if (scanResult >= 0) {
            Log.printf("[WiFi] Roaming scan complete. Found %d networks.\n", scanResult);
            _handleWiFiScanResults(true);
        } else if (scanResult == WIFI_SCAN_FAILED) {
            Log.println("[WiFi] Roaming scan failed.");
            WiFi.scanDelete();
            _wifiRoamingScanInProgress = false;
        }
    }

    // ── Wi-Fi active roaming trigger ─────────────────────────────────────
    if (wifiNow && !_wifiRoamingScanInProgress && !_wifiScanInProgress) {
        unsigned long now = millis();
        // Initialize _lastRssiCheckMs if it was 0 (just connected)
        if (_lastRssiCheckMs == 0) {
            _lastRssiCheckMs = now;
        }
        if (now - _lastRssiCheckMs >= 30000) {
            _lastRssiCheckMs = now;
            int32_t currentRSSI = WiFi.RSSI();
            if (currentRSSI < -75) {
                if (now - _lastScanMs >= 120000) {
                    Log.printf("[WiFi] RSSI (%d dBm) below threshold (-75 dBm). Triggering active roaming scan...\n", currentRSSI);
                    _lastScanMs = now;
                    _wifiRoamingScanInProgress = true;
                    WiFi.scanNetworks(true, false, false, 300, 0, WIFI_SSID);
                } else {
                    Log.println("[WiFi] RSSI is low, but active roaming scan is rate-limited (last scan < 120s ago).");
                }
            }
        }
    }

    // ── Wi-Fi reconnection ──────────────────────────────────────────────
    if (!wifiNow && !_wifiConnecting && !_wifiScanInProgress) {
        unsigned long now = millis();
        if (now - _wifiReconnectMs >= 5000) {
            _wifiReconnectMs = now;
            _connectWiFi();
        }
    }

    // Wi-Fi connection timeout — count as fault and allow retry
    if (_wifiConnecting && millis() - _wifiReconnectMs > 15000) {
        _wifiConnecting = false;
        _wifiFaults++;
        Log.printf("[WiFi] Connection timeout (total faults: %d)\n", _wifiFaults);
    }

    // Detect Wi-Fi connected and clear connecting flag
    if (WiFi.status() == WL_CONNECTED && _wifiConnecting) {
        _wifiConnecting = false;
        _lastRssiCheckMs = millis(); // Reset check timer on successful connection
    }

    // ── MQTT reconnection ───────────────────────────────────────────────
    if (WiFi.status() == WL_CONNECTED && !_mqttClient.connected() && !_mqttConnecting) {
        unsigned long now = millis();
        if (now - _mqttReconnectMs >= _mqttBackoffMs) {
            _mqttReconnectMs = now;
            _connectMQTT();
        }
    }

    // Handle MQTT connection timeout
    if (_mqttConnecting && millis() - _mqttReconnectMs > 5000) {
        _mqttConnecting = false;
        _mqttBackoffMs = min(_mqttBackoffMs * 2, 30000UL);
    }

    // ── MQTT RTT ping ───────────────────────────────────────────────────
    if (_mqttClient.connected()) {
        unsigned long nowMs = millis();
        // Timeout a pending ping after 5 s
        if (_rttPending && nowMs - _rttSentMs > 5000) {
            _rttPending = false;
            _rttLastMs = -1;
            Log.println("[MQTT] RTT ping timed out");
        }
        // Send a new ping every 30 s when not already waiting
        if (!_rttPending && nowMs >= _rttNextPingMs) {
            _pingRtt();
        }
    }
}

bool NetworkMgr::connected() {
    return WiFi.status() == WL_CONNECTED && _mqttClient.connected();
}

bool NetworkMgr::timeSynced() const {
    return time(nullptr) > 100000;  // NTP gives us time > 1973
}

bool NetworkMgr::publish(const char* topic, const char* payload) {
    if (!_mqttClient.connected()) return false;
    return _mqttClient.publish(topic, payload);
}

// ── Publish Helpers ─────────────────────────────────────────────────────────

void NetworkMgr::publishEvent(const char* eventType, int64_t timestamp,
                               int64_t probingStarted, int64_t activeAt,
                               float duration) {
    char buf[256];
    if (strcmp(eventType, "started") == 0) {
        snprintf(buf, sizeof(buf),
            "{\"event\":\"started\",\"probing_started_at\":%lld,"
            "\"active_at\":%lld,\"timestamp\":%lld}",
            (long long)probingStarted, (long long)activeAt, (long long)timestamp);
    } else {
        snprintf(buf, sizeof(buf),
            "{\"event\":\"ended\",\"ended_at\":%lld,\"duration\":%.1f,"
            "\"probing_started_at\":%lld,\"active_at\":%lld,\"timestamp\":%lld}",
            (long long)timestamp, (double)duration,
            (long long)probingStarted, (long long)activeAt, (long long)timestamp);
    }
    publish(MQTT_TOPIC_EVENT, buf);
}

void NetworkMgr::publishHeartbeat(unsigned long uptimeSec) {
    char buf[128];
    time_t now = time(nullptr);
    snprintf(buf, sizeof(buf),
        "{\"timestamp\":%lu,\"uptime_sec\":%lu,\"status\":\"running\"}",
        (unsigned long)now, uptimeSec);
    if (publish(MQTT_TOPIC_HEARTBEAT, buf)) {
        _lastHeartbeatSec = uptimeSec;
    }
}

// ── Helper: human-readable reset reason ────────────────────────────────────

static const char* resetReasonStr() {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:   return "power_on";
        case ESP_RST_EXT:       return "ext_pin";
        case ESP_RST_SW:        return "software";
        case ESP_RST_PANIC:     return "panic";
        case ESP_RST_INT_WDT:   return "int_watchdog";
        case ESP_RST_TASK_WDT:  return "task_watchdog";
        case ESP_RST_WDT:       return "other_watchdog";
        case ESP_RST_DEEPSLEEP: return "deep_sleep_wakeup";
        case ESP_RST_BROWNOUT:  return "brownout";
        case ESP_RST_SDIO:      return "sdio";
        default:                return "unknown";
    }
}

void NetworkMgr::publishTelemetry(unsigned long uptimeSec, int state, int eventsToday) {
    // ── System metrics ──────────────────────────────────────────────────
    time_t now = time(nullptr);

    // Internal temperature (raw Hall/temp sensor — fahr)
    float tempC = (temprature_sens_read() - 32) / 1.8f;

    // Wi-Fi radio info
    int rssi    = WiFi.RSSI();
    int channel = WiFi.channel();
    char bssid[18];
    const uint8_t* rawBssid = WiFi.BSSID();
    if (rawBssid) snprintf(bssid, sizeof(bssid), "%02X:%02X:%02X:%02X:%02X:%02X",
        rawBssid[0], rawBssid[1], rawBssid[2], rawBssid[3], rawBssid[4], rawBssid[5]);
    else strcpy(bssid, "00:00:00:00:00:00");

    // Heap
    size_t freeHeap = ESP.getFreeHeap();
    size_t minHeap  = ESP.getMinFreeHeap();

    // FreeRTOS task stats
    UBaseType_t taskCount = uxTaskGetNumberOfTasks();
    UBaseType_t fsmStack  = taskHandleFsm   ? uxTaskGetStackHighWaterMark(taskHandleFsm)   : 0;
    UBaseType_t audioStack= taskHandleAudio ? uxTaskGetStackHighWaterMark(taskHandleAudio) : 0;

    // ── Serialize ───────────────────────────────────────────────────────
    JsonDocument doc;
    doc["timestamp"]        = (unsigned long)now;
    doc["uptime_sec"]       = uptimeSec;
    doc["state"]            = state;
    doc["events_today"]     = eventsToday;

    // Wi-Fi
    doc["wifi_rssi"]        = rssi;
    doc["wifi_channel"]     = channel;
    doc["wifi_bssid"]       = bssid;
    doc["wifi_disconnects"] = _wifiDisconnects;
    doc["wifi_faults"]      = _wifiFaults;

    // MQTT
    doc["mqtt_reconnects"]  = _mqttReconnects;
    doc["mqtt_rtt_ms"]      = _rttLastMs;      // -1 = unknown

    // Memory
    doc["free_heap"]        = (unsigned int)freeHeap;
    doc["min_heap"]         = (unsigned int)minHeap;

    // Temperature
    char tempBuf[8];
    snprintf(tempBuf, sizeof(tempBuf), "%.1f", tempC);
    doc["internal_temp_c"]  = serialized(tempBuf);

    // Reset reason
    doc["reset_reason"]     = resetReasonStr();

    // FreeRTOS
    doc["task_count"]       = (unsigned int)taskCount;
    doc["fsm_stack_hwm"]    = (unsigned int)fsmStack;
    doc["audio_stack_hwm"]  = (unsigned int)audioStack;

    char buf[768];
    serializeJson(doc, buf, sizeof(buf));

    if (publish(MQTT_TOPIC_TELEMETRY, buf)) {
        _lastTelemetrySec = uptimeSec;
        Log.printf("[Telemetry] Published (temp=%.1f°C, rtt=%dms, tasks=%u)\n",
                      tempC, _rttLastMs, (unsigned)taskCount);
    }
}

// ── Wi-Fi ───────────────────────────────────────────────────────────────────

void NetworkMgr::_connectWiFi() {
    _wifiConnecting = true;
    _wifiReconnectMs = millis(); // Track the time connection attempt began
    _startWiFiScan();
}

void NetworkMgr::_startWiFiScan() {
    Log.printf("[WiFi] Starting asynchronous scan for SSID: %s\n", WIFI_SSID);
    WiFi.disconnect(true);
    _wifiScanInProgress = true;
    _wifiScanStartMs = millis();
    WiFi.scanNetworks(true, false, false, 300, 0, WIFI_SSID);
}

void NetworkMgr::_handleWiFiScanResults(bool isRoaming) {
    int16_t n = WiFi.scanComplete();
    _wifiScanInProgress = false;
    _wifiRoamingScanInProgress = false;

    if (n <= 0) {
        Log.println("[WiFi] No matching APs found in scan results.");
        WiFi.scanDelete();
        if (!isRoaming) {
            // Fallback to standard connection
            _wifiConnecting = true;
            _wifiReconnectMs = millis();
            if (strlen(WIFI_EAP_USERNAME) > 0) {
                Log.println("[WiFi] Connecting to default SSID (WPA2-Enterprise)");
                WiFi.begin(WIFI_SSID, WIFI_EAP_METHOD, WIFI_EAP_IDENTITY, WIFI_EAP_USERNAME, WIFI_EAP_PASSWORD);
            } else {
                Log.println("[WiFi] Connecting to default SSID (WPA2-PSK)");
                esp_wifi_sta_wpa2_ent_disable();
                WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            }
        }
        return;
    }

    int bestNetworkIdx = -1;
    int32_t bestRSSI = -999;
    
    for (int i = 0; i < n; i++) {
        if (WiFi.SSID(i) == WIFI_SSID) {
            int32_t rssi = WiFi.RSSI(i);
            if (rssi > bestRSSI) {
                bestRSSI = rssi;
                bestNetworkIdx = i;
            }
        }
    }

    if (bestNetworkIdx == -1) {
        Log.println("[WiFi] No AP with matching SSID found in results.");
        WiFi.scanDelete();
        if (!isRoaming) {
            // Fallback to standard connection
            _wifiConnecting = true;
            _wifiReconnectMs = millis();
            if (strlen(WIFI_EAP_USERNAME) > 0) {
                Log.println("[WiFi] Connecting to default SSID (WPA2-Enterprise)");
                WiFi.begin(WIFI_SSID, WIFI_EAP_METHOD, WIFI_EAP_IDENTITY, WIFI_EAP_USERNAME, WIFI_EAP_PASSWORD);
            } else {
                Log.println("[WiFi] Connecting to default SSID (WPA2-PSK)");
                esp_wifi_sta_wpa2_ent_disable();
                WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            }
        }
        return;
    }

    uint8_t bestBssid[6];
    memcpy(bestBssid, WiFi.BSSID(bestNetworkIdx), 6);
    int32_t bestChannel = WiFi.channel(bestNetworkIdx);
    
    Log.printf("[WiFi] Selected best AP: %02X:%02X:%02X:%02X:%02X:%02X (Channel: %d, RSSI: %d dBm)\n",
               bestBssid[0], bestBssid[1], bestBssid[2], bestBssid[3], bestBssid[4], bestBssid[5],
               bestChannel, bestRSSI);

    if (isRoaming) {
        int32_t currentRSSI = WiFi.RSSI();
        const uint8_t* currentBssid = WiFi.BSSID();
        
        bool isSameAp = false;
        if (currentBssid) {
            isSameAp = (memcmp(currentBssid, bestBssid, 6) == 0);
        }

        if (isSameAp) {
            Log.println("[WiFi] Best AP is already the currently connected AP. No roam needed.");
        } else if (bestRSSI >= currentRSSI + 8) {
            Log.printf("[WiFi] Roaming: Current RSSI = %d dBm, New AP RSSI = %d dBm. Roaming to new BSSID...\n",
                       currentRSSI, bestRSSI);
            
            WiFi.disconnect(false);
            _wifiConnecting = true;
            _wifiReconnectMs = millis();

            if (strlen(WIFI_EAP_USERNAME) > 0) {
                WiFi.begin(WIFI_SSID, WIFI_EAP_METHOD, WIFI_EAP_IDENTITY, WIFI_EAP_USERNAME, WIFI_EAP_PASSWORD);
                _applyBssidAndChannel(bestBssid, bestChannel);
            } else {
                esp_wifi_sta_wpa2_ent_disable();
                WiFi.begin(WIFI_SSID, WIFI_PASSWORD, bestChannel, bestBssid);
            }
        } else {
            Log.printf("[WiFi] Found stronger AP (%d dBm) but difference from current (%d dBm) is less than +8 dBm hysteresis. No roam.\n",
                       bestRSSI, currentRSSI);
        }
    } else {
        _wifiConnecting = true;
        _wifiReconnectMs = millis();

        if (strlen(WIFI_EAP_USERNAME) > 0) {
            WiFi.begin(WIFI_SSID, WIFI_EAP_METHOD, WIFI_EAP_IDENTITY, WIFI_EAP_USERNAME, WIFI_EAP_PASSWORD);
            _applyBssidAndChannel(bestBssid, bestChannel);
        } else {
            esp_wifi_sta_wpa2_ent_disable();
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD, bestChannel, bestBssid);
        }
    }

    WiFi.scanDelete();
}

void NetworkMgr::_applyBssidAndChannel(const uint8_t* bssid, int channel) {
    wifi_config_t wifi_cfg;
    if (esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg) == ESP_OK) {
        wifi_cfg.sta.bssid_set = true;
        memcpy(wifi_cfg.sta.bssid, bssid, 6);
        if (channel > 0) {
            wifi_cfg.sta.channel = (uint8_t)channel;
        }
        esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    }
}

// ── MQTT ────────────────────────────────────────────────────────────────────

void NetworkMgr::_connectMQTT() {
    _mqttConnecting = true;
    _mqttReconnectMs = millis();

    Log.print("[MQTT] Connecting to ");
    Log.print(MQTT_SERVER);
    Log.print(":");
    Log.println(MQTT_PORT);

    // Set the callback before connecting
    _mqttClient.setCallback(staticMqttCallback);

    // Try to connect with a unique client ID
    char clientId[24];
    snprintf(clientId, sizeof(clientId), "aap_%08X", (unsigned int)ESP.getEfuseMac());

    bool hasMqttCredentials = strlen(MQTT_USERNAME) > 0;
    bool connected = hasMqttCredentials
        ? _mqttClient.connect(clientId, MQTT_USERNAME, MQTT_PASSWORD)
        : _mqttClient.connect(clientId);

    if (connected) {
        Log.println("[MQTT] Connected");
        _mqttConnecting = false;
        _mqttBackoffMs = 1000;

        _mqttClient.subscribe(MQTT_TOPIC_CONFIG);
        Log.print("[MQTT] Subscribed to "); Log.println(MQTT_TOPIC_CONFIG);

        _mqttClient.subscribe(MQTT_TOPIC_CONFIG_REQ);
        Log.print("[MQTT] Subscribed to "); Log.println(MQTT_TOPIC_CONFIG_REQ);

        _mqttClient.subscribe(MQTT_TOPIC_RTT);
        Log.print("[MQTT] Subscribed to "); Log.println(MQTT_TOPIC_RTT);

        _mqttClient.subscribe(MQTT_TOPIC_RESTART);
        Log.print("[MQTT] Subscribed to "); Log.println(MQTT_TOPIC_RESTART);

        _mqttClient.subscribe(MQTT_TOPIC_TELEMETRY_REQ);
        Log.print("[MQTT] Subscribed to "); Log.println(MQTT_TOPIC_TELEMETRY_REQ);

        // Backdate timers so heartbeat + telemetry fire immediately after connect
        unsigned long uptimeSec = millis() / 1000;
        _lastHeartbeatSec = (uptimeSec >= HEARTBEAT_INTERVAL_SEC) ? uptimeSec - HEARTBEAT_INTERVAL_SEC : 0;
        _lastTelemetrySec = (uptimeSec >= TELEMETRY_INTERVAL_SEC) ? uptimeSec - TELEMETRY_INTERVAL_SEC : 0;

        // Kick off first RTT ping after 5 s
        _rttNextPingMs = millis() + 5000;
    } else {
        Log.print("[MQTT] Failed (rc=");
        Log.print(_mqttClient.state());
        Log.println(")");
        _mqttConnecting = false;
        _mqttBackoffMs = min(_mqttBackoffMs * 2, 30000UL);
        ++_mqttReconnects;
    }
}

// ── MQTT RTT ─────────────────────────────────────────────────────────────────

void NetworkMgr::_pingRtt() {
    unsigned long now = millis();
    char payload[24];
    snprintf(payload, sizeof(payload), "%lu", now);
    if (_mqttClient.publish(MQTT_TOPIC_RTT, payload)) {
        _rttPending = true;
        _rttSentMs  = now;
        _rttNextPingMs = now + 30000;
    }
}

// ── MQTT Message Callback ────────────────────────────────────────────────────

void NetworkMgr::_mqttCallback(char* topic, byte* payload, unsigned int length) {
    char buf[1024];
    unsigned int copyLen = min(length, (unsigned int)sizeof(buf) - 1);
    memcpy(buf, payload, copyLen);
    buf[copyLen] = '\0';

    Log.print("[MQTT] Message received on topic: ");
    Log.println(topic);

    if (strcmp(topic, MQTT_TOPIC_RTT) == 0) {
        // Echo back: payload is the millisecond timestamp we sent
        if (_rttPending) {
            unsigned long sentMs = (unsigned long)atoll(buf);
            unsigned long nowMs  = millis();
            if (nowMs >= sentMs) {
                _rttLastMs = (int)(nowMs - sentMs);
                Log.printf("[MQTT] RTT = %d ms\n", _rttLastMs);
            }
            _rttPending = false;
        }
    } else if (strcmp(topic, MQTT_TOPIC_CONFIG_REQ) == 0) {
        if (_configRequestCb) {
            _configRequestCb();
        }
    } else if (strcmp(topic, MQTT_TOPIC_CONFIG) == 0) {
        if (_configCb) {
            _configCb(buf);
        }
    } else if (strcmp(topic, MQTT_TOPIC_RESTART) == 0) {
        Log.println("[MQTT] Restart command received. Rebooting ESP32...");
        delay(500);
        ESP.restart();
    } else if (strcmp(topic, MQTT_TOPIC_TELEMETRY_REQ) == 0) {
        if (_telemetryRequestCb) {
            _telemetryRequestCb();
        }
    }
}

void NetworkMgr::publishConfigAck(const RuntimeConfig& cfg) {
    JsonDocument doc;
    doc["main_snr_db"]          = cfg.main_snr_threshold;
    doc["sec_snr_db"]           = cfg.sec_snr_threshold;
    doc["confirm_sec"]          = cfg.confirm_sec;
    doc["probing_timeout_sec"]  = cfg.probing_timeout_sec;
    doc["active_timeout_sec"]   = cfg.active_timeout_sec;
    doc["pulse_on_sec"]         = cfg.pulse_on_sec;
    doc["pulse_off_sec"]        = cfg.pulse_off_sec;
    doc["pulse_tolerance_sec"]  = cfg.pulse_tolerance_sec;
    doc["cycle_target_ms"]      = cfg.cycle_target_ms;
    doc["cycle_tolerance_ms"]   = cfg.cycle_tolerance_ms;
    doc["required_cycles"]      = cfg.required_cycles;
    doc["signal_streak_min"]    = cfg.signal_streak_min;
    doc["main_freq_hz"]         = cfg.main_freq_hz;
    doc["sec_freq_hz"]          = cfg.sec_freq_hz;
    doc["amb_freq_hz"]          = cfg.amb_freq_hz;
    doc["alpha_attack"]         = cfg.alpha_attack;
    doc["alpha_decay"]          = cfg.alpha_decay;
    doc["deep_sleep_enabled"]   = cfg.deep_sleep_enabled;
    doc["sleep_start_hour"]     = cfg.sleep_start_hour;
    doc["wake_end_hour"]        = cfg.wake_end_hour;
    doc["led_enabled"]          = cfg.led_enabled;
    doc["ota_enabled"]          = cfg.ota_enabled;
    doc["ota_port"]             = cfg.ota_port;
    doc["debug_enabled"]        = cfg.debug_enabled;
    doc["mqtt_logs_enabled"]    = cfg.mqtt_logs_enabled;
    doc["udp_stream_enabled"]   = cfg.udp_stream_enabled;
    doc["udp_host"]             = cfg.udp_host;
    doc["udp_port"]             = cfg.udp_port;

    char buf[1024];
    serializeJson(doc, buf, sizeof(buf));
    publish(MQTT_TOPIC_CONFIG_ACK, buf);
    Log.println("[MQTT] Published config acknowledgement");
}