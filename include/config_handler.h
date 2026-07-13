#pragma once

struct RuntimeConfig;

/// Handles incoming MQTT config payloads and config-request callbacks.
/// Registered with NetworkMgr::onConfig() / onConfigRequest() in setup().
void configCallback(const char* json);
void configRequestCallback();
void telemetryRequestCallback();

/// NVS Persistence Helpers
void loadConfigFromNVS(RuntimeConfig& cfg);
void saveConfigToNVS(const RuntimeConfig& cfg);

