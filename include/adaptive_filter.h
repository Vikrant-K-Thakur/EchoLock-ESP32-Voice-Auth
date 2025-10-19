#ifndef ADAPTIVE_FILTER_H
#define ADAPTIVE_FILTER_H

#include <stdint.h>
#include <Arduino.h>
#include "noise_analysis.h"    // from Satyam's module (NoiseMetrics, NoiseAnalyzer)

/*
 Adaptive Filtering Engine
 -------------------------
 Role: Utkarsh (Person 3)
 Purpose: Dynamically reduce noise & enhance speech using adaptive filter selection.
*/

enum class FilterMode : uint8_t {
    BYPASS = 0,
    LPF,
    HPF,
    BPF,
    SMOOTH,        // moving average filter
    ADAPTIVE_NLG  // adaptive noise-level gain suppression
};

struct FilterConfig {
    uint32_t sample_rate = 16000;
    uint16_t frame_size  = 256;

    // Biquad parameters
    float lpf_cutoff = 3400.0f;
    float hpf_cutoff = 300.0f;
    float bpf_center = 1200.0f;
    float bpf_q      = 0.7f;

    // Smoothing filter
    uint8_t smooth_taps = 5;

    // Adaptive gain suppression
    float nlg_gain_factor = 1.0f;
    float nlg_min_gain = 0.1f;

    // Crossfade parameters
    uint8_t crossfade_frames = 3;
};

class AdaptiveFilter {
public:
    AdaptiveFilter();
    ~AdaptiveFilter();

    void init(const FilterConfig& cfg);
    void updateFromNoiseMetrics(const NoiseMetrics& m);
    void setMode(FilterMode mode);
    void processFrame(const int16_t* input, int16_t* output);
    FilterMode getMode() const { return active_mode_; }
    void printState();
    void setConfig(const FilterConfig& cfg);

private:
    // ------- Internal Classes --------
    struct Biquad {
        float a0, a1, a2, b0, b1, b2;
        float x1, x2, y1, y2;
        Biquad();
        void reset();
        void setCoeffs(float B0, float B1, float B2, float A0, float A1, float A2);
        float processSample(float x);
        void copyStateFrom(const Biquad& other);
    };

    struct Smooth {
        uint8_t taps;
        float* ring;
        uint16_t idx;
        float sum;
        void init(uint8_t t);
        void reset();
        float process(float in);
        void deinit();
    };

    // ------- Internal Methods --------
    void rebuildTargetFilter();
    void startCrossfadeTo(FilterMode newMode);
    // Removed: void doCrossfadeStep(); // Logic handled in processFrame

    float computeFrameEnergy(const int16_t* frame);
    void applyNLG(const int16_t* in, int16_t* out, float frameEnergy, const NoiseMetrics& m);

    // ------- Filter Design Helpers --------
    static void designBiquadLPF(Biquad& bq, float fs, float fc, float Q = 0.707f);
    static void designBiquadHPF(Biquad& bq, float fs, float fc, float Q = 0.707f);
    static void designBiquadBPF(Biquad& bq, float fs, float f0, float Q);

    // ------- Members --------
    FilterConfig cfg_;
    bool initialized_;
    FilterMode active_mode_;
    FilterMode target_mode_;
    Biquad active_bq_, target_bq_;
    Smooth smooth_filter_;
    uint8_t crossfade_total_frames_, crossfade_remaining_, crossfade_current_frame_;
    NoiseMetrics last_metrics_;
    float* scratch_float_buf_;
};

#endif