#include "ota.h"
#include "config.h"
#include <Arduino.h>
#include <ArduinoOTA.h>

// ── OTA Initialisation ────────────────────────────────────────────────────────

void initOTA() {
    String hostname = "aap-node-";
    hostname += String((uint32_t)(ESP.getEfuseMac() & 0xFFFFFF), HEX);

    ArduinoOTA.setHostname(hostname.c_str());
    ArduinoOTA.setPort(g_config.ota_port);

    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Log.printf("[OTA] Start: %s\n", type.c_str());
    });

    ArduinoOTA.onEnd([]() {
        Log.println("[OTA] Done");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Log.printf("[OTA] Progress: %u%%\r", progress / (total / 100));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Log.printf("[OTA] Error %u: ", error);
        if      (error == OTA_AUTH_ERROR)    Log.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)   Log.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Log.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Log.println("Receive Failed");
        else if (error == OTA_END_ERROR)     Log.println("End Failed");
    });

    if (g_config.ota_enabled) {
        ArduinoOTA.begin();
        Log.printf("[OTA] Ready on port %d (hostname: %s)\n",
                      g_config.ota_port, hostname.c_str());
    } else {
        Log.println("[OTA] Disabled by config");
    }
}
