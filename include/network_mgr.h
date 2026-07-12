#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"

class NetworkMgr {
public:
    NetworkMgr();
    void begin();
    void loop();

    // Publish helpers
    bool publish(const char* topic, const char* payload);
    void publishEvent(const char* eventType, int64_t timestamp,
                      int64_t probingStarted, int64_t activeAt,
                      float duration = 0.0f);
    void publishHeartbeat(unsigned long uptimeSec);
    void publishTelemetry(int rssi, int wifiSnr, size_t freeHeap,
                          size_t minHeap, unsigned long uptimeSec,
                          int state, int eventsToday, int mqttReconnects);
    void publishConfigAck(const RuntimeConfig& cfg);

    // Status
    bool connected();
    bool timeSynced() const;
    unsigned long lastHeartbeatSec() const { return _lastHeartbeatSec; }
    unsigned long lastTelemetrySec() const { return _lastTelemetrySec; }
    int mqttReconnects() const { return _mqttReconnects; }

    // Config callback
    using ConfigCallback = void (*)(const char* jsonPayload);
    void onConfig(ConfigCallback cb) { _configCb = cb; }

    using ConfigRequestCallback = void (*)();
    void onConfigRequest(ConfigRequestCallback cb) { _configRequestCb = cb; }

    // Static MQTT callback dispatcher
    static void staticMqttCallback(char* topic, byte* payload, unsigned int length);

private:
    WiFiClient _wifiClient;
    PubSubClient _mqttClient;

    // Singleton pointer for static callback dispatch
    static NetworkMgr* _instance;

    // Wi-Fi state
    bool _wifiConnecting;
    unsigned long _wifiReconnectMs;

    // MQTT state
    bool _mqttConnecting;
    unsigned long _mqttReconnectMs;
    unsigned long _mqttBackoffMs;
    int _mqttReconnects;

    // Timers
    unsigned long _lastHeartbeatSec;
    unsigned long _lastTelemetrySec;

    // Config callback
    ConfigCallback _configCb;
    ConfigRequestCallback _configRequestCb;

    // Internal methods
    void _connectWiFi();
    void _connectMQTT();
    void _mqttCallback(char* topic, byte* payload, unsigned int length);
};