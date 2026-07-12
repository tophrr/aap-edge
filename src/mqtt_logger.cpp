#include "mqtt_logger.h"
#include "config.h"
#include <string.h>

QueueHandle_t g_logQueue = nullptr;

MqttLogger::MqttLogger() : _bufferIdx(0) {
    memset(_lineBuffer, 0, sizeof(_lineBuffer));
}

size_t MqttLogger::write(uint8_t c) {
    // Always write to physical serial interface
    Serial.write(c);

    if (g_config.mqtt_logs_enabled) {
        if (c == '\n') {
            _lineBuffer[_bufferIdx] = '\0';
            _sendToQueue();
            _bufferIdx = 0;
        } else if (c != '\r') { // Ignore carriage return
            if (_bufferIdx < sizeof(_lineBuffer) - 1) {
                _lineBuffer[_bufferIdx++] = c;
            }
        }
    }
    return 1;
}

size_t MqttLogger::write(const uint8_t *buffer, size_t size) {
    // Always write to physical serial interface
    Serial.write(buffer, size);

    if (g_config.mqtt_logs_enabled) {
        for (size_t i = 0; i < size; i++) {
            char c = (char)buffer[i];
            if (c == '\n') {
                _lineBuffer[_bufferIdx] = '\0';
                _sendToQueue();
                _bufferIdx = 0;
            } else if (c != '\r') {
                if (_bufferIdx < sizeof(_lineBuffer) - 1) {
                    _lineBuffer[_bufferIdx++] = c;
                }
            }
        }
    }
    return size;
}

void MqttLogger::_sendToQueue() {
    if (g_logQueue == nullptr) {
        g_logQueue = xQueueCreate(32, sizeof(LogMessage)); // Allocate a queue for 32 log messages
        if (g_logQueue == nullptr) return;
    }
    
    // Ignore empty lines
    if (strlen(_lineBuffer) == 0) return;

    LogMessage msg;
    strncpy(msg.text, _lineBuffer, sizeof(msg.text) - 1);
    msg.text[sizeof(msg.text) - 1] = '\0';

    // Non-blocking write to queue (0 ticks timeout)
    xQueueSend(g_logQueue, &msg, 0);
}

// Global logger instance
MqttLogger Log;
