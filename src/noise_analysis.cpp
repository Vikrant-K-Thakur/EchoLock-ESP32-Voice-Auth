#include "noise_analysis.h"
#include <math.h> // For log10f, sqrt, cos, sin
#include <Arduino.h>

// CRITICAL FIX: Define M_PI for portability, as it is not guaranteed by standard headers.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Constructor
NoiseAnalyzer::NoiseAnalyzer() 
    : sample_rate_(16000), frame_size_(256), noise_floor_(100.0f), initialized_(false) {
}

// Initialization
void NoiseAnalyzer::init(uint16_t sample_rate, uint16_t frame_size) {
    sample_rate_ = sample_rate;
    frame_size_ = frame_size;
    noise_floor_ = 50.0f; // Initial conservative estimate of power
    initialized_ = true;
    
    Serial.printf("[NoiseAnalysis] Initialized - Sample Rate: %d, Frame Size: %d\n", 
                  sample_rate_, frame_size_);
}

// Main analysis function - called by other modules
NoiseMetrics NoiseAnalyzer::analyze(const int16_t* audio_buffer, uint16_t buffer_size) {
    NoiseMetrics metrics;
    
    if (!initialized_ || buffer_size == 0) {
        // Return default values if not initialized
        metrics.snr_db = 0.0f;
        metrics.zcr = 0.0f;
        metrics.is_noise_dominant = true;
        metrics.spectral_centroid = 0.0f;
        metrics.noise_floor = noise_floor_;
        return metrics;
    }
    
    // Update noise floor estimation
    estimateNoiseFloor(audio_buffer, buffer_size);
    
    // Compute all metrics
    metrics.snr_db = computeSNR(audio_buffer, buffer_size);
    metrics.zcr = computeZCR(audio_buffer, buffer_size);
    // CRITICAL FIX: Pass buffer_size to spectral centroid
    metrics.spectral_centroid = computeSpectralCentroid(audio_buffer, buffer_size);
    metrics.is_noise_dominant = isNoiseDominant(audio_buffer, buffer_size);
    metrics.noise_floor = noise_floor_;
    
    return metrics;
}

// Compute Signal-to-Noise Ratio in dB
float NoiseAnalyzer::computeSNR(const int16_t* audio_buffer, uint16_t buffer_size) {
    // Calculate signal power (RMS Power)
    float sum_squares = 0.0f;
    for (uint16_t i = 0; i < buffer_size; i++) {
        // Use float multiplication for better precision than long long
        sum_squares += (float)audio_buffer[i] * (float)audio_buffer[i];
    }
    
    float signal_power = sum_squares / buffer_size;
    
    // Avoid log(0) and handle very small values
    if (signal_power < 1.0f) signal_power = 1.0f; // Use 1.0f as floor to avoid deep negative dB for silence
    if (noise_floor_ < 1.0f) noise_floor_ = 1.0f;
    
    // Convert power ratio to dB
    float snr = 10.0f * log10f(signal_power / noise_floor_);
    
    // Clamp SNR to reasonable range
    if (snr < -20.0f) snr = -20.0f;
    if (snr > 60.0f) snr = 60.0f;
    
    return snr;
}

// Compute Zero Crossing Rate
float NoiseAnalyzer::computeZCR(const int16_t* audio_buffer, uint16_t buffer_size) {
    if (buffer_size < 2) return 0.0f;
    
    uint16_t zero_crossings = 0;
    
    for (uint16_t i = 1; i < buffer_size; i++) {
        // ZCR definition: sign change from sample i-1 to i
        if ((audio_buffer[i-1] >= 0 && audio_buffer[i] < 0) || 
            (audio_buffer[i-1] < 0 && audio_buffer[i] >= 0)) {
            zero_crossings++;
        }
    }
    
    return (float)zero_crossings / (buffer_size - 1);
}

// Apply Hamming window for FFT
void NoiseAnalyzer::applyHammingWindow(float* windowed_buffer, const int16_t* input_buffer, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        double window = 0.54 - 0.46 * cos(2 * M_PI * i / (size - 1));
        windowed_buffer[i] = (float)input_buffer[i] * (float)window;
    }
}

// Compute FFT (Simplified DFT Implementation - WARNING: TOO SLOW FOR REAL-TIME AUDIO)
void NoiseAnalyzer::computeFFT(const int16_t* audio_buffer, uint16_t N, float* magnitude_output) {
    // WARNING: This is a computationally expensive O(N^2) Discrete Fourier Transform (DFT).
    // For real-time applications, use a highly optimized O(N log N) Fast Fourier Transform (FFT) library.
    
    if (N == 0) return;
    float windowed[N];
    
    // Apply windowing
    applyHammingWindow(windowed, audio_buffer, N);
    
    // Compute magnitude spectrum (simplified)
    for (uint16_t k = 0; k < N/2; k++) {
        float real = 0.0f;
        float imag = 0.0f;
        
        for (uint16_t n = 0; n < N; n++) {
            float angle = 2.0f * M_PI * (float)k * (float)n / (float)N;
            real += windowed[n] * cosf(angle);
            imag -= windowed[n] * sinf(angle);
        }
        
        // Magnitude normalized by N
        magnitude_output[k] = sqrtf(real * real + imag * imag) / N;
    }
}

// Compute Spectral Centroid
float NoiseAnalyzer::computeSpectralCentroid(const int16_t* audio_buffer, uint16_t buffer_size) {
    // NOTE: This assumes buffer_size is power-of-two if using a true FFT library
    uint16_t N = buffer_size;
    
    // The magnitudes array must be large enough to hold N/2 + 1 elements, but N/2 is sufficient here
    float magnitudes[N/2]; 
    computeFFT(audio_buffer, N, magnitudes);
    
    float weighted_sum = 0.0f;
    float sum_magnitudes = 0.0f;
    
    for (uint16_t i = 0; i < N/2; i++) {
        // Frequency bin calculation: freq = (bin_index * sample_rate) / N
        float freq = (float)i * sample_rate_ / (float)N;
        weighted_sum += freq * magnitudes[i];
        sum_magnitudes += magnitudes[i];
    }
    
    if (sum_magnitudes < 1e-6f) return 0.0f;
    return weighted_sum / sum_magnitudes;
}

// Noise dominance detection
bool NoiseAnalyzer::isNoiseDominant(const int16_t* audio_buffer, uint16_t buffer_size) {
    // Recalculate or retrieve metrics (they are calculated in analyze(), but this is fine)
    float snr = computeSNR(audio_buffer, buffer_size);
    float zcr = computeZCR(audio_buffer, buffer_size);
    // CRITICAL FIX: Pass buffer_size
    float spectral_centroid = computeSpectralCentroid(audio_buffer, buffer_size);
    
    // Decision logic based on multiple features
    bool low_snr = snr < 10.0f; // Adjusted threshold slightly for typical speech
    bool high_zcr = zcr > 0.35f; // Adjusted threshold slightly for typical unvoiced sounds/hiss
    bool low_centroid = spectral_centroid < 1200.0f; // Adjusted for 16kHz sample rate
    
    // Weighted decision (adjust weights based on testing)
    int score = 0;
    if (low_snr) score += 3;
    if (high_zcr) score += 1;
    if (low_centroid) score += 1;
    
    return score >= 3; // Threshold for noise dominance (low SNR is the strongest indicator)
}

// Adaptive noise floor estimation
void NoiseAnalyzer::estimateNoiseFloor(const int16_t* audio_buffer, uint16_t buffer_size) {
    // Simple moving average for noise floor estimation (RMS Power)
    float current_power = 0.0f;
    for (uint16_t i = 0; i < buffer_size; i++) {
        current_power += (float)audio_buffer[i] * (float)audio_buffer[i];
    }
    current_power /= buffer_size;
    
    // Update noise floor adaptively (slow adaptation)
    // Only update *downwards* or when power is low/stable (likely noise)
    if (current_power < noise_floor_ || current_power < noise_floor_ * 1.5f) {
        // Slow update rate (0.05f) to prevent speech from raising the floor too quickly
        noise_floor_ = 0.95f * noise_floor_ + 0.05f * current_power;
    }
    
    // Ensure noise floor doesn't go too low or below the noise in the microphone itself
    if (noise_floor_ < 100.0f) noise_floor_ = 100.0f; 
}

// Debug function to print metrics
void NoiseAnalyzer::printMetrics(const NoiseMetrics& metrics) {
    Serial.printf("[NoiseAnalysis] SNR: %.2f dB, ZCR: %.3f, Centroid: %.1f Hz, ", 
                  metrics.snr_db, metrics.zcr, metrics.spectral_centroid);
    Serial.printf("NoiseFloor: %.1f, Dominant: %s\n", 
                  metrics.noise_floor, metrics.is_noise_dominant ? "NOISE" : "SPEECH");
}