#include "config.h"
#include "fsm.h"
#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>

// ── I2S Initialization ──────────────────────────────────────────────────────

static i2s_config_t _i2sConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
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
        Log.print("[I2S] Driver install failed: ");
        Log.println(err);
        abort();
    }

    err = i2s_set_pin(I2S_PORT, &_i2sPins);
    if (err != ESP_OK) {
        Log.print("[I2S] Pin config failed: ");
        Log.println(err);
        abort();
    }

    i2s_zero_dma_buffer(I2S_PORT);

    Log.println("[I2S] Initialized: " + String(SAMPLE_RATE) + " Hz, " + String(I2S_BITS_PER_SAMPLE_32BIT) + "-bit mono, INMP441");
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
    static int32_t rawSamples[NUM_SAMPLES];
    static float floatSamples[NUM_SAMPLES];
    size_t bytesRead = 0;

    Log.println("[AudioDSP] Task started on Core 0");

    while (true) {
        // Read block of samples from I2S DMA
        esp_err_t err = i2s_read(I2S_PORT, rawSamples, sizeof(rawSamples),
                                  &bytesRead, portMAX_DELAY);
        if (err != ESP_OK) {
            Log.print("[AudioDSP] i2s_read error: ");
            Log.println(err);
            continue;
        }

        int samplesRead = bytesRead / sizeof(int32_t);
        if (samplesRead < NUM_SAMPLES) continue;

        // ── Raw sample diagnostic (first frame only) ──────────────────
        if (g_debugEnabled) {
            static bool rawDiagDone = false;
            if (!rawDiagDone) {
                int nonzero = 0;
                for (int i = 0; i < samplesRead; ++i) {
                    if (rawSamples[i] != 0) ++nonzero;
                }
                Log.printf("[DSP] Raw samples (32-bit): %d total, %d non-zero, first 16: ",
                    samplesRead, nonzero);
                for (int i = 0; i < min(16, samplesRead); ++i) {
                    Log.printf("0x%08X ", rawSamples[i]);
                }
                DEBUG_PRINTLN("");
                rawDiagDone = true;
            }
        }

        // Convert raw I2S words to float [-1.0, 1.0].
        // INMP441 sends 24-bit standard I2S samples that appear in the top
        // portion of each 32-bit word after the 1-bit data delay.
        for (int i = 0; i < samplesRead; ++i) {
            int32_t sample24 = rawSamples[i] >> 8;  // Shift to get 24-bit value
            floatSamples[i] = (float)sample24 / 8388608.0f;  // Normalize to [-1.0, 1.0]
        }

        // Apply Hann window
        applyHann(floatSamples, samplesRead);

        // Compute Goertzel magnitudes
        float main_db = goertzelDB(floatSamples, samplesRead, g_config.main_freq_hz, SAMPLE_RATE);
        float sec_db  = goertzelDB(floatSamples, samplesRead, g_config.sec_freq_hz, SAMPLE_RATE);
        float amb_db  = goertzelDB(floatSamples, samplesRead, g_config.amb_freq_hz, SAMPLE_RATE);

        // Apply asymmetric Exponential Moving Average (EMA) for ambient noise baseline
        static float smoothed_amb_db = 0.0f;
        static bool amb_initialized = false;
        if (!amb_initialized) {
            smoothed_amb_db = amb_db;
            amb_initialized = true;
        } else {
            if (amb_db > smoothed_amb_db) {
                // Environment is getting louder -> slow reaction (attack)
                smoothed_amb_db += g_config.alpha_attack * (amb_db - smoothed_amb_db);
            } else {
                // Environment is getting quieter -> fast reaction (decay)
                smoothed_amb_db += g_config.alpha_decay * (amb_db - smoothed_amb_db);
            }
        }

        // ── Verbose debug: keep prints bounded so they don't starve the DSP task ─
        if (g_debugEnabled) {
            static int dspFrame = 0;
            static unsigned long lastDspPrintMs = 0;
            dspFrame++;
            float main_snr = main_db - smoothed_amb_db;
            float sec_snr = sec_db - smoothed_amb_db;
            // Raw sample stats (useful for verifying mic is working)
            int32_t rawMin = rawSamples[0];
            int32_t rawMax = rawSamples[0];

            float sumSq = 0;
            for (int i = 0; i < samplesRead; ++i) {
                sumSq += floatSamples[i] * floatSamples[i];
                if (rawSamples[i] < rawMin) rawMin = rawSamples[i];
                if (rawSamples[i] > rawMax) rawMax = rawSamples[i];
            }
            float rms = sqrtf(sumSq / samplesRead);
            float rms_db = 20.0f * log10f(rms + 1e-12f);
            unsigned long nowMs = millis();
            bool shouldPrint = dspFrame <= 20 ||
                (nowMs - lastDspPrintMs) >= DEBUG_PRINT_INTERVAL_MS ||
                main_snr > 0.0f || sec_snr > 0.0f;
            if (shouldPrint) {
                lastDspPrintMs = nowMs;
                Log.printf("[DSP] #%d | RMS=%.1fdB raw=[%d,%d] | main=%.1f sec=%.1f amb=%.1f (raw=%.1f) | main_snr=%.1f sec_snr=%.1f\n",
                    dspFrame, (double)rms_db, rawMin, rawMax,
                    (double)main_db, (double)sec_db, (double)smoothed_amb_db, (double)amb_db,
                    (double)main_snr, (double)sec_snr);
            }
        }

        // Send to FSM task via queue
        AudioFrame frame = { main_db, sec_db, smoothed_amb_db };
        if (xQueueSend(audioQueue, &frame, pdMS_TO_TICKS(10)) != pdPASS) {
            // Queue full — drop frame (non-critical)
            static int dropCount = 0;
            if (++dropCount % 100 == 0) {
                Log.printf("[AudioDSP] Dropped %d frames (queue full)\n", dropCount);
            }
        }
    }
}