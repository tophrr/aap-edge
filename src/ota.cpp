#include "ota.h"
#include "config.h"
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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

// ── Remote OTA Task ─────────────────────────────────────────────────────────

static void remoteOTATask(void* parameter) {
    String url = (char*)parameter;
    free(parameter);

    Log.printf("[RemoteOTA] Starting download from: %s\n", url.c_str());

    // Configure callbacks for HTTPUpdate
    httpUpdate.onStart([]() {
        Log.println("[RemoteOTA] Update Started");
    });
    httpUpdate.onEnd([]() {
        Log.println("[RemoteOTA] Update Finished, restarting...");
    });
    httpUpdate.onProgress([](int current, int total) {
        static int lastPercent = 0;
        int percent = (current * 100) / total;
        if (percent != lastPercent && percent % 10 == 0) {
            Log.printf("[RemoteOTA] Progress: %d%%\n", percent);
            lastPercent = percent;
        }
    });
    httpUpdate.onError([](int err) {
        Log.printf("[RemoteOTA] Error[%d]: %s\n", err, httpUpdate.getLastErrorString().c_str());
    });

    t_httpUpdate_return ret;

    if (url.startsWith("https://")) {
        WiFiClientSecure clientSecure;
        clientSecure.setInsecure();
        ret = httpUpdate.update(clientSecure, url);
    } else {
        WiFiClient client;
        ret = httpUpdate.update(client, url);
    }

    switch (ret) {
        case HTTP_UPDATE_FAILED:
            Log.printf("[RemoteOTA] HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
            break;
        case HTTP_UPDATE_NO_UPDATES:
            Log.println("[RemoteOTA] HTTP_UPDATE_NO_UPDATES");
            break;
        case HTTP_UPDATE_OK:
            Log.println("[RemoteOTA] HTTP_UPDATE_OK");
            ESP.restart();
            break;
    }

    // Delete self if it fails
    vTaskDelete(NULL);
}

void startRemoteOTA(const char* url) {
    if (!g_config.ota_enabled) {
        Log.println("[RemoteOTA] Blocked: OTA is disabled in config");
        return;
    }
    
    // Copy URL to heap so it survives until task reads it
    char* urlCopy = strdup(url);
    if (!urlCopy) {
        Log.println("[RemoteOTA] Failed to allocate memory for URL");
        return;
    }

    // Create a high priority task to perform the update
    xTaskCreate(
        remoteOTATask,
        "remoteOTA",
        8192,
        (void*)urlCopy,
        10, // high priority
        NULL
    );
}
