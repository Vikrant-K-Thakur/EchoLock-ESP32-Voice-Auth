#include <Arduino.h>
#include "i2s_interface.h"
#include "authentication.h"
#include "adaptive_filter.h"
#include "noise_analysis.h"
#include "ui_feedback.h"

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

// Forward declarations
void onShortPress();
void onLongPress();

// --- Initialize I2S ---
void setup_i2s() {
    if (!I2SInterface::init()) {
        Serial.println("❌ I2S initialization failed!");
        while (1);
    }
    Serial.println("✅ I2S initialized successfully");
}

// Helper function to constrain values
template <typename T>
T constrain_val(T val, T min_val, T max_val) {
    return (val < min_val) ? min_val : (val > max_val) ? max_val : val;
}

// --- Simulated Audio Capture (since real mic not connected) ---
void capture_audio_data(int16_t *buffer, size_t count, bool isNoisy = false) {
    for (size_t i = 0; i < count; i++) {
        if (isNoisy) {
            // Simulate speech in heavy, high-frequency noise
            float speech = 5000 * sin(2 * PI * i / 20);
            float noise = 2000 * sin(2 * PI * i / 5 + random(-500, 500)); // High freq noise
            buffer[i] = (int16_t)constrain_val(speech + noise, -32768.0f, 32767.0f);
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
    UIFeedback::init();
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
    // Real-time voice testing loop
    UIFeedback::loopPoll(onShortPress, onLongPress);
    delay(50);
}

// Button callbacks for real testing
void onShortPress() {
    Serial.println("\n[USER] Short press - Starting Authentication...");
    UIFeedback::indicateListening();
    
    int16_t audioBuffer[DEMO_FRAME_SIZE];
    
    // Capture real audio from microphone
    size_t samplesRead = I2SInterface::readSamples(audioBuffer, DEMO_FRAME_SIZE);
    
    if (samplesRead > 0) {
        UIFeedback::indicateAnalyzing();
        
        // Process with adaptive filter
        NoiseMetrics metrics = noiseAnalyzer.analyze(audioBuffer, samplesRead);
        adaptiveFilter.updateFromNoiseMetrics(metrics);
        
        int16_t filteredAudio[DEMO_FRAME_SIZE];
        adaptiveFilter.processFrame(audioBuffer, filteredAudio);
        
        // Authenticate
        auto features = extractFeatures(filteredAudio, samplesRead);
        String matchedUser;
        bool authenticated = authenticateUser(features, matchedUser);
        
        if (authenticated) {
            UIFeedback::indicateAccessGranted();
            Serial.println("✅ Access Granted: " + matchedUser);
        } else {
            UIFeedback::indicateAccessDenied();
            Serial.println("❌ Access Denied");
        }
        
        delay(2000);
        UIFeedback::resetSystem();
    }
}

void onLongPress() {
    Serial.println("\n[USER] Long press - Starting Enrollment...");
    UIFeedback::indicateEnrollStart();
    
    int16_t audioBuffer[DEMO_FRAME_SIZE];
    
    // Capture enrollment audio
    size_t samplesRead = I2SInterface::readSamples(audioBuffer, DEMO_FRAME_SIZE);
    
    if (samplesRead > 0) {
        auto features = extractFeatures(audioBuffer, samplesRead);
        bool enrolled = enrollUser("User1", features);
        
        if (enrolled) {
            UIFeedback::indicateEnrollDone();
            Serial.println("✅ User enrolled successfully");
        } else {
            UIFeedback::indicateAccessDenied();
            Serial.println("❌ Enrollment failed");
        }
        
        delay(2000);
        UIFeedback::resetSystem();
    }
}