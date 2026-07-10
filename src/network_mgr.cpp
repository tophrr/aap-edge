#include "network_mgr.h"
#include "config.h"
#include <esp_wifi.h>
#include <esp_eap_client.h>

// ── Constructor ─────────────────────────────────────────────────────────────

NetworkMgr* NetworkMgr::_instance = nullptr;

NetworkMgr::NetworkMgr()
    : _mqttClient(_wifiClient)
    , _wifiConnecting(false)
    , _wifiReconnectMs(0)
    , _mqttConnecting(false)
    , _mqttReconnectMs(0)
    , _mqttBackoffMs(1000)
    , _mqttReconnects(0)
    , _lastHeartbeatSec(0)
    , _lastTelemetrySec(0)
    , _configCb(nullptr)
{
    _mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    _mqttClient.setBufferSize(512);
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
    Serial.println("[Network] Starting Wi-Fi...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true, true);
    _connectWiFi();
}

void NetworkMgr::loop() {
    // Handle MQTT client loop
    if (_mqttClient.connected()) {
        _mqttClient.loop();
    }

    // ── Wi-Fi reconnection ──────────────────────────────────────────────
    if (WiFi.status() != WL_CONNECTED && !_wifiConnecting) {
        unsigned long now = millis();
        if (now - _wifiReconnectMs >= 5000) {
            _wifiReconnectMs = now;
            _connectWiFi();
        }
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
}

bool NetworkMgr::connected() const {
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

void NetworkMgr::publishEvent(const char* eventType, float timestamp,
                               float probingStarted, float activeAt,
                               float duration) {
    char buf[256];
    if (strcmp(eventType, "started") == 0) {
        snprintf(buf, sizeof(buf),
            "{\"event\":\"started\",\"probing_started_at\":%.1f,"
            "\"active_at\":%.1f,\"timestamp\":%.1f}",
            (double)probingStarted, (double)activeAt, (double)timestamp);
    } else {
        snprintf(buf, sizeof(buf),
            "{\"event\":\"ended\",\"ended_at\":%.1f,\"duration\":%.1f,"
            "\"probing_started_at\":%.1f,\"active_at\":%.1f,\"timestamp\":%.1f}",
            (double)timestamp, (double)duration,
            (double)probingStarted, (double)activeAt, (double)timestamp);
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

void NetworkMgr::publishTelemetry(int rssi, int wifiSnr, size_t freeHeap,
                                   size_t minHeap, unsigned long uptimeSec,
                                   int state, int eventsToday, int mqttReconnects) {
    char buf[256];
    time_t now = time(nullptr);
    snprintf(buf, sizeof(buf),
        "{\"timestamp\":%lu,\"wifi_rssi\":%d,\"wifi_snr\":%d,"
        "\"free_heap\":%u,\"min_heap\":%u,\"uptime_sec\":%lu,"
        "\"state\":%d,\"events_today\":%d,\"mqtt_reconnects\":%d,"
        "\"boot_count\":1}",
        (unsigned long)now, rssi, wifiSnr,
        freeHeap, minHeap, uptimeSec,
        state, eventsToday, mqttReconnects);
    if (publish(MQTT_TOPIC_TELEMETRY, buf)) {
        _lastTelemetrySec = uptimeSec;
    }
}

// ── Wi-Fi ───────────────────────────────────────────────────────────────────

void NetworkMgr::_connectWiFi() {
    _wifiConnecting = true;
    Serial.print("[WiFi] Connecting to ");
    Serial.print(WIFI_SSID);

    if (strlen(WIFI_EAP_USERNAME) > 0) {
        Serial.println(" (WPA2-Enterprise)");
        esp_eap_client_set_identity((uint8_t*)WIFI_EAP_IDENTITY, strlen(WIFI_EAP_IDENTITY));
        esp_eap_client_set_username((uint8_t*)WIFI_EAP_USERNAME, strlen(WIFI_EAP_USERNAME));
        esp_eap_client_set_password((uint8_t*)WIFI_EAP_PASSWORD, strlen(WIFI_EAP_PASSWORD));
        esp_wifi_sta_enterprise_enable();
        WiFi.begin(WIFI_SSID);
    } else {
        Serial.println(" (WPA2-PSK)");
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
}

// ── MQTT ────────────────────────────────────────────────────────────────────

void NetworkMgr::_connectMQTT() {
    _mqttConnecting = true;
    _mqttReconnectMs = millis();
    _mqttBackoffMs = 1000;

    Serial.print("[MQTT] Connecting to ");
    Serial.print(MQTT_SERVER);
    Serial.print(":");
    Serial.println(MQTT_PORT);

    // Set the callback before connecting
    _mqttClient.setCallback(staticMqttCallback);

    // Try to connect with a unique client ID
    char clientId[24];
    snprintf(clientId, sizeof(clientId), "aap_%08X", (unsigned int)ESP.getEfuseMac());

    if (_mqttClient.connect(clientId)) {
        Serial.println("[MQTT] Connected");
        _mqttConnecting = false;
        _mqttBackoffMs = 1000;

        // Subscribe to config topic
        _mqttClient.subscribe(MQTT_TOPIC_CONFIG);
        Serial.print("[MQTT] Subscribed to ");
        Serial.println(MQTT_TOPIC_CONFIG);
    } else {
        Serial.print("[MQTT] Failed (rc=");
        Serial.print(_mqttClient.state());
        Serial.println(")");
        _mqttConnecting = false;
        _mqttBackoffMs = min(_mqttBackoffMs * 2, 30000UL);
        ++_mqttReconnects;
    }
}

void NetworkMgr::_mqttCallback(char* topic, byte* payload, unsigned int length) {
    // Null-terminate the payload
    char buf[512];
    unsigned int copyLen = min(length, (unsigned int)sizeof(buf) - 1);
    memcpy(buf, payload, copyLen);
    buf[copyLen] = '\0';

    Serial.print("[MQTT] Config received: ");
    Serial.println(buf);

    if (_configCb) {
        _configCb(buf);
    }
}