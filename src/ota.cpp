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
        Serial.printf("[OTA] Start: %s\n", type.c_str());
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("[OTA] Done");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("[OTA] Progress: %u%%\r", progress / (total / 100));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] Error %u: ", error);
        if      (error == OTA_AUTH_ERROR)    Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)     Serial.println("End Failed");
    });

    if (g_config.ota_enabled) {
        ArduinoOTA.begin();
        Serial.printf("[OTA] Ready on port %d (hostname: %s)\n",
                      g_config.ota_port, hostname.c_str());
    } else {
        Serial.println("[OTA] Disabled by config");
    }
}
