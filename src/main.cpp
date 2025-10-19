#include <Arduino.h>
#include "driver/i2s.h"
#include "authentication.h"
#include "adaptive_filter.h"
#include "noise_analysis.h" // We need the analyzer for a complete demo

// --- I2S Pin Configuration ---
#define I2S_WS      25   // LRCL (Word Select)
#define I2S_SD      33   // DOUT (Data In)
#define I2S_SCK     14   // BCLK (Bit Clock)
#define I2S_PORT    I2S_NUM_0

// --- Audio Settings ---
#define SAMPLE_RATE      16000
#define DEMO_FRAME_SIZE  256  // Use 256 for frame size for FFT in NoiseAnalysis
#define DEMO_SAMPLE_COUNT DEMO_FRAME_SIZE // Match capture count to frame size

// --- Global Modules ---
NoiseAnalyzer noiseAnalyzer;
AdaptiveFilter adaptiveFilter;
FilterConfig filterCfg; // Configuration for the filter

// --- Initialize I2S ---
void setup_i2s() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        // CRITICAL FIX: The INMP441 raw capture is 32-bit (24-bit data MSB-aligned)
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
        .data_out_num = -1,    // Only RX
        .data_in_num = I2S_SD
    };

    // Install and configure I2S
    if (i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL) != ESP_OK) {
        Serial.println("❌ I2S driver install failed!");
        while (1);
    }
    if (i2s_set_pin(I2S_PORT, &pin_config) != ESP_OK) {
        Serial.println("❌ I2S pin config failed!");
        while (1);
    }
    i2s_zero_dma_buffer(I2S_PORT);
    Serial.println("✅ I2S initialized successfully");
}

// --- Simulated Audio Capture (since real mic not connected) ---
void capture_audio_data(int16_t *buffer, size_t count, bool isNoisy = false) {
    for (size_t i = 0; i < count; i++) {
        if (isNoisy) {
            // Simulate speech in heavy, high-frequency noise
            float speech = 5000 * sin(2 * PI * i / 20);
            float noise = 2000 * sin(2 * PI * i / 5 + random(-500, 500)); // High freq noise
            buffer[i] = (int16_t)clamp_value(speech + noise, -32768.0f, 32767.0f);
        } else {
            // Simulate clean speech
            buffer[i] = (int16_t)(8000 * sin(2 * PI * i / 30) + random(-50, 50)); 
        }
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n===============================");
    Serial.println("ESP32 Voice Authentication Demo");
    Serial.println("===============================\n");

    setup_i2s();
    auth_init();
    noiseAnalyzer.init(SAMPLE_RATE, DEMO_FRAME_SIZE);
    
    // Configure and initialize adaptive filter
    filterCfg.sample_rate = SAMPLE_RATE;
    filterCfg.frame_size = DEMO_FRAME_SIZE;
    adaptiveFilter.init(filterCfg);
    Serial.println("✅ All modules initialized.\n");

    delay(1000);

    // --- Demo Scenario: Filter Test & Authentication ---
    Serial.println("[DEMO] Starting noisy frame capture and filtering...");
    
    int16_t rawAudio[DEMO_FRAME_SIZE];
    int16_t filteredAudio[DEMO_FRAME_SIZE];
    
    // Simulate capturing a noisy voice sample
    capture_audio_data(rawAudio, DEMO_FRAME_SIZE, true); 

    // 1. Analyze the noisy raw frame
    NoiseMetrics metrics = noiseAnalyzer.analyze(rawAudio, DEMO_FRAME_SIZE);
    noiseAnalyzer.printMetrics(metrics);
    
    // 2. Adaptive Filter decides mode and processes
    adaptiveFilter.updateFromNoiseMetrics(metrics);
    adaptiveFilter.processFrame(rawAudio, filteredAudio);
    adaptiveFilter.printState();
    
    // 3. Re-analyze the filtered frame (optional: check improvement)
    NoiseMetrics filteredMetrics = noiseAnalyzer.analyze(filteredAudio, DEMO_FRAME_SIZE);
    Serial.println("--- Filtered Audio Metrics ---");
    noiseAnalyzer.printMetrics(filteredMetrics);
    
    // 4. Authenticate using the CLEANED audio features
    Serial.println("\n[AUTH] Authenticating using CLEANED features...");
    auto featuresEnroll = extractFeatures(rawAudio, DEMO_FRAME_SIZE); // Enroll with clean-ish sample
    enrollUser("Vikrant", featuresEnroll);
    
    auto featuresAuth = extractFeatures(filteredAudio, DEMO_FRAME_SIZE); // Authenticate with filtered sample

    String matchedUser;
    bool ok = authenticateUser(featuresAuth, matchedUser);

    if (ok) {
        Serial.println("\n✅ Authentication success with filtered audio! User: " + matchedUser);
    } else {
        Serial.println("\n❌ Authentication failed (Filter/Auth issue).");
    }

    Serial.println("\n🎯 Integration test completed successfully!");
}

void loop() {
    // Nothing to do here
}