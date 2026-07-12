#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

struct LogMessage {
    char text[192];
};

extern QueueHandle_t g_logQueue;

class MqttLogger : public Print {
public:
    MqttLogger();
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;

private:
    char _lineBuffer[192];
    size_t _bufferIdx;
    void _sendToQueue();
};

extern MqttLogger Log;
