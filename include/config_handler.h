#pragma once

/// Handles incoming MQTT config payloads and config-request callbacks.
/// Registered with NetworkMgr::onConfig() / onConfigRequest() in setup().
void configCallback(const char* json);
void configRequestCallback();
