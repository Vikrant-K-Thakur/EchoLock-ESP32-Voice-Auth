#include "i2s_interface.h"
#include <Arduino.h>
#include "driver/i2s.h"

// ============================
// I2S Pin Configuration (CRITICAL FIX: PINS CORRECTED to match wiring GPIO25/33)
// ============================
#define I2S_WS      25  // LRCL (Word Select) -> CORRECTED from 15 to 25
#define I2S_SD      33  // DOUT (Data) -> CORRECTED from 32 to 33
#define I2S_SCK     14  // BCLK (Bit Clock)
#define I2S_PORT    I2S_NUM_0

// ============================
// I2S Settings
// ============================
#define SAMPLE_RATE     16000
#define DMA_BUF_COUNT   4
#define DMA_BUF_LEN     512

// ============================
// I2S Initialization
// ============================
void I2SInterface::init() {
    // I2S Configuration structure
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // Mic outputs 24-bit in 32-bit frame
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // Mono (left channel, as L/R is tied to GND)
        // Use standard I2S format. The MSB flag is redundant and deprecated.
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = DMA_BUF_COUNT,
        .dma_buf_len = DMA_BUF_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    // Pin configuration
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = -1,      // not used (RX only)
        .data_in_num = I2S_SD
    };

    // Install and start I2S driver
    esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("[I2S] Driver install failed: %d\n", err);
        return;
    }
    i2s_set_pin(I2S_PORT, &pin_config);
    i2s_zero_dma_buffer(I2S_PORT);

    Serial.println("[I2S] Interface initialized successfully.");
}

// ============================
// I2S Audio Capture
// ============================
int I2SInterface::readAudio(int16_t *samples, size_t num_samples) {
    if (!samples || num_samples == 0) return 0;

    // Use a temporary buffer to hold the 32-bit I2S frames
    int32_t i2s_buffer[num_samples];
    size_t bytes_read = 0;

    // Read data from I2S
    esp_err_t result = i2s_read(I2S_PORT, (void*)i2s_buffer, sizeof(i2s_buffer), &bytes_read, portMAX_DELAY);
    if (result != ESP_OK || bytes_read == 0) {
        // Serial.println("[I2S] Read failed or no data."); // Commented out for cleaner loop output
        return 0;
    }

    size_t samples_read = bytes_read / sizeof(int32_t);

    // Convert 32-bit samples (24-bit data, MSB-aligned) to 16-bit PCM
    // CRITICAL FIX: The total required shift is 16 bits (32 bits frame - 16 bits target = 16)
    for (size_t i = 0; i < samples_read && i < num_samples; i++) {
        // Shifting right by 16 discards the 8 LSBs of the 32-bit frame (padding)
        // AND converts the remaining 24-bit value down to 16-bit resolution.
        samples[i] = (int16_t)(i2s_buffer[i] >> 16);
    }

    return samples_read;
}

// ============================
// Optional: Microphone Test Function
// ============================
void I2SInterface::testMic() {
    const size_t N = 512;
    int16_t samples[N];
    int read = readAudio(samples, N);

    if (read > 0) {
        // Compute basic RMS to verify mic input
        double sum_sq = 0;
        for (int i = 0; i < read; i++) sum_sq += (double)samples[i] * (double)samples[i];
        double rms = sqrt(sum_sq / read);

        // Normalize the RMS for a 0-255 scale (based on 16-bit max amplitude 32767)
        double max_amplitude = 32768.0; 
        uint8_t metric = (uint8_t)min(255.0, (rms / max_amplitude) * 255.0 * 2.0); // Factor of 2.0 for visibility

        Serial.printf("[I2S] Read %d samples | RMS = %.2f | M = %d\n", read, rms, metric);
    } else {
        Serial.println("[I2S] No samples read.");
    }
}
