#include <Arduino.h>
#include "i2s_interface.h"
#include "authentication.h"
#include "adaptive_filter.h"
#include "noise_analysis.h"
#include "ui_feedback.h"
#include "FS.h"
#include "SPIFFS.h"

// --- I2S Pin Configuration ---
#define I2S_WS      25   // LRCL (Word Select)
#define I2S_SD      33   // DOUT (Data In)
#define I2S_SCK     14   // BCLK (Bit Clock)
#define I2S_PORT    I2S_NUM_0

// --- Audio Settings ---
#define SAMPLE_RATE      16000
#define DEMO_FRAME_SIZE  256  
#define DEMO_SAMPLE_COUNT DEMO_FRAME_SIZE 

// --- Global Modules ---
NoiseAnalyzer noiseAnalyzer;
AdaptiveFilter adaptiveFilter;
FilterConfig filterCfg;

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

// --- Simulated Audio Capture (used when no mic connected) ---
void capture_audio_data(int16_t *buffer, size_t count, bool isNoisy = false, bool saveToFile = false) {
    // Mount SPIFFS if needed
    if (saveToFile && !SPIFFS.begin(true)) {
        Serial.println("[ERROR] SPIFFS Mount Failed");
        saveToFile = false;
    }

    Serial.println("[AUDIO] Recording started... Speak now!");

    for (size_t i = 0; i < count; i++) {
        if (isNoisy) {
            float speech = 5000 * sin(2 * PI * i / 20);
            float noise = 2000 * sin(2 * PI * i / 5 + random(-500, 500));
            buffer[i] = (int16_t)constrain_val(speech + noise, -32768.0f, 32767.0f);
        } else {
            buffer[i] = (int16_t)(8000 * sin(2 * PI * i / 30) + random(-50, 50)); 
        }

        // 🟢 Print live samples to Serial
        Serial.print(buffer[i]);
        Serial.print(" ");
        if (i % 100 == 0) Serial.println();
    }

    Serial.println("\n[AUDIO] Recording complete.\n");

    // 💾 Save the voice data for enrollment
    if (saveToFile) {
        File file = SPIFFS.open("/user1_voice.raw", FILE_WRITE);
        if (!file) {
            Serial.println("[ERROR] Failed to open file for writing!");
        } else {
            file.write((uint8_t *)buffer, count * sizeof(int16_t));
            file.close();
            Serial.printf("[INFO] Voice sample saved to /user1_voice.raw (%u samples)\n", count);
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

    // Mount SPIFFS for saving voice data
    if (!SPIFFS.begin(true)) {
        Serial.println("[ERROR] SPIFFS Mount Failed!");
    } else {
        Serial.println("✅ SPIFFS Mounted Successfully");
    }

    setup_i2s();
    auth_init();
    UIFeedback::init();
    noiseAnalyzer.init(SAMPLE_RATE, DEMO_FRAME_SIZE);

    filterCfg.sample_rate = SAMPLE_RATE;
    filterCfg.frame_size = DEMO_FRAME_SIZE;
    adaptiveFilter.init(filterCfg);
    Serial.println("✅ All modules initialized.\n");

    delay(1000);

    Serial.println("[DEMO] Starting initial noise filtering test...");

    static int16_t rawAudio[DEMO_FRAME_SIZE];
    static int16_t filteredAudio[DEMO_FRAME_SIZE];

    capture_audio_data(rawAudio, DEMO_FRAME_SIZE, true);

    NoiseMetrics metrics = noiseAnalyzer.analyze(rawAudio, DEMO_FRAME_SIZE);
    noiseAnalyzer.printMetrics(metrics);

    adaptiveFilter.updateFromNoiseMetrics(metrics);
    adaptiveFilter.processFrame(rawAudio, filteredAudio);
    adaptiveFilter.printState();

    NoiseMetrics filteredMetrics = noiseAnalyzer.analyze(filteredAudio, DEMO_FRAME_SIZE);
    Serial.println("--- Filtered Audio Metrics ---");
    noiseAnalyzer.printMetrics(filteredMetrics);

    Serial.println("\n[DEMO] Pre-enrolling demo user...");
    auto featuresEnroll = extractFeatures(rawAudio, DEMO_FRAME_SIZE);
    enrollUser("DemoUser", featuresEnroll);

    auto featuresAuth = extractFeatures(filteredAudio, DEMO_FRAME_SIZE);
    String matchedUser;
    bool ok = authenticateUser(featuresAuth, matchedUser);

    if (ok)
        Serial.println("\n✅ Authentication success with filtered audio! User: " + matchedUser);
    else
        Serial.println("\n❌ Authentication failed (filter/auth issue).");

    Serial.println("\n🎯 Initialization test completed!");
}

void loop() {
    UIFeedback::loopPoll(onShortPress, onLongPress);
    delay(50);
}

// --- Short Press (Authentication) ---
void onShortPress() {
    Serial.println("\n[USER] Short press - Starting Authentication...");
    UIFeedback::indicateListening();

    static int16_t audioBuffer[DEMO_FRAME_SIZE];
    size_t samplesRead = I2SInterface::readSamples(audioBuffer, DEMO_FRAME_SIZE);

    // 🟢 Print captured samples live
    Serial.println("[AUTH] Live audio samples:");
    for (size_t i = 0; i < samplesRead; i++) {
        Serial.print(audioBuffer[i]);
        Serial.print(" ");
        if (i % 100 == 0) Serial.println();
    }

    if (samplesRead > 0) {
        UIFeedback::indicateAnalyzing();
        delay(1500);

        NoiseMetrics metrics = noiseAnalyzer.analyze(audioBuffer, samplesRead);
        adaptiveFilter.updateFromNoiseMetrics(metrics);

        static int16_t filteredAudio[DEMO_FRAME_SIZE];
        adaptiveFilter.processFrame(audioBuffer, filteredAudio);
        delay(1000);

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
        delay(3000);
        UIFeedback::resetSystem();
    }
}

// --- Long Press (Enrollment) ---
void onLongPress() {
    Serial.println("\n[USER] Long press - Starting Enrollment...");
    UIFeedback::indicateEnrollStart();

    delay(1500);
    Serial.println("[ENROLL] Recording voice... Speak now!");
    UIFeedback::showMessage("Speak Now!");

    static int16_t audioBuffer[DEMO_FRAME_SIZE];
    size_t samplesRead = I2SInterface::readSamples(audioBuffer, DEMO_FRAME_SIZE);

    // 🟢 Print all captured samples on Serial
    Serial.println("[ENROLL] Live audio samples:");
    for (size_t i = 0; i < samplesRead; i++) {
        Serial.print(audioBuffer[i]);
        Serial.print(" ");
        if (i % 100 == 0) Serial.println();
    }

    // 💾 Save to SPIFFS for persistent storage
    File file = SPIFFS.open("/user1_voice.raw", FILE_WRITE);
    if (!file) {
        Serial.println("[ERROR] Failed to open file for saving voice data!");
    } else {
        file.write((uint8_t *)audioBuffer, samplesRead * sizeof(int16_t));
        file.close();
        Serial.printf("[INFO] Voice sample saved to /user1_voice.raw (%u samples)\n", samplesRead);
    }

    Serial.println("[ENROLL] Voice captured, analyzing...");
    UIFeedback::showMessage("Analyzing...");
    delay(2000);

    if (samplesRead > 0) {
        auto features = extractFeatures(audioBuffer, samplesRead);
        bool enrolled = enrollUser("User1", features);

        if (enrolled) {
            UIFeedback::indicateEnrollDone();
            Serial.println("✅ User enrolled successfully!");
        } else {
            UIFeedback::indicateEnrollFailed();
            Serial.println("❌ Enrollment failed.");
        }
    } else {
        UIFeedback::indicateEnrollFailed();
        Serial.println("❌ No audio captured.");
    }

    delay(3000);
    UIFeedback::resetSystem();
}
