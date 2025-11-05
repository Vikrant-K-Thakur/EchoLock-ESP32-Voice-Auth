#include "i2s_interface.h"
#include <Arduino.h>

bool I2SInterface::initialized_ = false;

bool I2SInterface::init() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 512,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = -1,
        .data_in_num = I2S_SD
    };

    if (i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL) != ESP_OK) {
        return false;
    }
    if (i2s_set_pin(I2S_PORT, &pin_config) != ESP_OK) {
        return false;
    }
    
    i2s_zero_dma_buffer(I2S_PORT);
    initialized_ = true;
    return true;
}

size_t I2SInterface::readSamples(int16_t* buffer, size_t count) {
    if (!initialized_) return 0;
    
    size_t bytes_read = 0;
    int32_t* raw_buffer = (int32_t*)malloc(count * sizeof(int32_t));
    
    if (i2s_read(I2S_PORT, raw_buffer, count * sizeof(int32_t), &bytes_read, portMAX_DELAY) == ESP_OK) {
        // Convert 32-bit to 16-bit (INMP441 uses MSB-aligned 24-bit in 32-bit)
        for (size_t i = 0; i < count; i++) {
            buffer[i] = (int16_t)(raw_buffer[i] >> 16);
        }
    }
    
    free(raw_buffer);
    return bytes_read / sizeof(int32_t);
}

void I2SInterface::cleanup() {
    if (initialized_) {
        i2s_driver_uninstall(I2S_PORT);
        initialized_ = false;
    }
}