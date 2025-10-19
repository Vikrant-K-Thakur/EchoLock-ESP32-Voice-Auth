#ifndef NOISE_ANALYSIS_H
#define NOISE_ANALYSIS_H

#include <stdint.h>

// Noise analysis results structure
struct NoiseMetrics {
    float snr_db;           // Signal-to-Noise Ratio in decibels
    float zcr;              // Zero Crossing Rate
    bool is_noise_dominant; // True if noise is louder than speech
    float spectral_centroid;// "Center of mass" of frequencies
    float noise_floor;      // Estimated background noise level
};

class NoiseAnalyzer {
public:
    // Constructor and initialization
    NoiseAnalyzer();
    // The frame_size argument defines the number of samples used for FFT/ZCR/SNR computation.
    void init(uint16_t sample_rate = 16000, uint16_t frame_size = 256);
    
    // Main analysis function
    NoiseMetrics analyze(const int16_t* audio_buffer, uint16_t buffer_size);
    
    // Individual metric functions
    float computeSNR(const int16_t* audio_buffer, uint16_t buffer_size);
    float computeZCR(const int16_t* audio_buffer, uint16_t buffer_size);
    // FFT and Centroid methods now take buffer_size to ensure correct sizing
    void computeFFT(const int16_t* audio_buffer, uint16_t buffer_size, float* magnitude_output);
    float computeSpectralCentroid(const int16_t* audio_buffer, uint16_t buffer_size);
    bool isNoiseDominant(const int16_t* audio_buffer, uint16_t buffer_size);
    
    // Configuration
    void setNoiseFloor(float floor) { noise_floor_ = floor; }
    float getNoiseFloor() const { return noise_floor_; }
    
    // Debug functions
    void printMetrics(const NoiseMetrics& metrics);
    
private:
    uint16_t sample_rate_;
    uint16_t frame_size_;
    float noise_floor_;
    bool initialized_;
    
    void estimateNoiseFloor(const int16_t* audio_buffer, uint16_t buffer_size);
    void applyHammingWindow(float* windowed_buffer, const int16_t* input_buffer, uint16_t size);
};

#endif