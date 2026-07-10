#include "config.h"
#include "fsm.h"
#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>

// ── I2S Initialization ──────────────────────────────────────────────────────

static i2s_config_t _i2sConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
};

static i2s_pin_config_t _i2sPins = {
    .bck_io_num = I2S_SCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD_PIN
};

void initI2S() {
    esp_err_t err = i2s_driver_install(I2S_PORT, &_i2sConfig, 0, NULL);
    if (err != ESP_OK) {
        Serial.print("[I2S] Driver install failed: ");
        Serial.println(err);
        abort();
    }

    err = i2s_set_pin(I2S_PORT, &_i2sPins);
    if (err != ESP_OK) {
        Serial.print("[I2S] Pin config failed: ");
        Serial.println(err);
        abort();
    }

    Serial.println("[I2S] Initialized: 8000 Hz, 16-bit mono, INMP441");
}

// ── Goertzel Algorithm ──────────────────────────────────────────────────────

static float goertzelPower(const float* samples, int numSamples,
                            float targetFreq, float sampleRate) {
    float coeff = 2.0f * cosf(2.0f * M_PI * targetFreq / sampleRate);
    float sPrev = 0.0f;
    float sPrev2 = 0.0f;

    for (int i = 0; i < numSamples; ++i) {
        float s = samples[i] + coeff * sPrev - sPrev2;
        sPrev2 = sPrev;
        sPrev = s;
    }

    float power = sPrev2 * sPrev2 + sPrev * sPrev - coeff * sPrev * sPrev2;
    return fmaxf(0.0f, power);
}

static float goertzelDB(const float* samples, int numSamples,
                         float targetFreq, float sampleRate) {
    float power = goertzelPower(samples, numSamples, targetFreq, sampleRate);
    return 20.0f * log10f(sqrtf(power) / (float)numSamples + 1e-12f);
}

// ── Hann Window ─────────────────────────────────────────────────────────────

static void applyHann(float* samples, int numSamples) {
    for (int i = 0; i < numSamples; ++i) {
        float hann = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (numSamples - 1)));
        samples[i] *= hann;
    }
}

// ── FreeRTOS Task ───────────────────────────────────────────────────────────

void audioDspTask(void* parameter) {
    QueueHandle_t audioQueue = (QueueHandle_t)parameter;
    int16_t rawSamples[NUM_SAMPLES];
    float floatSamples[NUM_SAMPLES];
    size_t bytesRead = 0;

    Serial.println("[AudioDSP] Task started on Core 0");

    while (true) {
        // Read block of samples from I2S DMA
        esp_err_t err = i2s_read(I2S_PORT, rawSamples, sizeof(rawSamples),
                                  &bytesRead, portMAX_DELAY);
        if (err != ESP_OK) {
            Serial.print("[AudioDSP] i2s_read error: ");
            Serial.println(err);
            continue;
        }

        int samplesRead = bytesRead / sizeof(int16_t);
        if (samplesRead < 32) continue;

        // Convert int16 to float [-1.0, 1.0]
        for (int i = 0; i < samplesRead; ++i) {
            floatSamples[i] = (float)rawSamples[i] / 32768.0f;
        }

        // Apply Hann window
        applyHann(floatSamples, samplesRead);

        // Compute Goertzel magnitudes
        float main_db = goertzelDB(floatSamples, samplesRead, MAIN_FREQ_HZ, SAMPLE_RATE);
        float sec_db  = goertzelDB(floatSamples, samplesRead, SEC_FREQ_HZ, SAMPLE_RATE);
        float amb_db  = goertzelDB(floatSamples, samplesRead, AMB_FREQ_HZ, SAMPLE_RATE);

        // Send to FSM task via queue
        AudioFrame frame = { main_db, sec_db, amb_db };
        if (xQueueSend(audioQueue, &frame, pdMS_TO_TICKS(10)) != pdPASS) {
            // Queue full — drop frame (non-critical)
            static int dropCount = 0;
            if (++dropCount % 100 == 0) {
                Serial.printf("[AudioDSP] Dropped %d frames (queue full)\n", dropCount);
            }
        }
    }
}