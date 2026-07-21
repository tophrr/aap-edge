#pragma once

/// Initialise ArduinoOTA and start listening if OTA is enabled in g_config.
void initOTA();

/// Start a remote OTA update from the given URL.
void startRemoteOTA(const char* url);
