#include "adaptive_filter.h"
#include <math.h>
#include <cstring> // For memcpy, memset

// CRITICAL FIX: Define PI for portability
#ifndef PI
#define PI 3.14159265358979323846f
#endif

// Helper function to constrain a value (replaces Arduino's constrain macro)
template <typename T>
T clamp_value(T val, T min_val, T max_val) {
    return (val < min_val) ? min_val : (val > max_val) ? max_val : val;
}

// Helper to round to nearest integer and cast to int16_t safely
int16_t float_to_int16(float f) {
    // Ensure value is within 16-bit signed range before rounding
    f = clamp_value(f, -32768.0f, 32767.0f);
    return (int16_t)lrintf(f); // lrintf is standard in C99 math
}


// ======================================================
//    Biquad Implementation
// ======================================================
AdaptiveFilter::Biquad::Biquad() { reset(); }

void AdaptiveFilter::Biquad::reset() {
    a0 = a1 = a2 = b0 = b1 = b2 = 0.0f;
    x1 = x2 = y1 = y2 = 0.0f;
}

void AdaptiveFilter::Biquad::setCoeffs(float b0_coeff, float b1_coeff, float b2_coeff, float a0_coeff, float a1_coeff, float a2_coeff) {
    // Normalize coefficients by a0_coeff
    if (fabs(a0_coeff) < 1e-9f) a0_coeff = 1.0f;
    b0 = b0_coeff / a0_coeff;
    b1 = b1_coeff / a0_coeff;
    b2 = b2_coeff / a0_coeff;
    a0 = 1.0f;
    a1 = a1_coeff / a0_coeff;
    a2 = a2_coeff / a0_coeff;
}

float AdaptiveFilter::Biquad::processSample(float x) {
    // Direct Form 1 (DF1) Transposed structure: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    float y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2;
    x2 = x1; x1 = x;
    y2 = y1; y1 = y;
    return y;
}

void AdaptiveFilter::Biquad::copyStateFrom(const Biquad& o) {
    x1=o.x1; x2=o.x2; y1=o.y1; y2=o.y2;
}

// ======================================================
//    Smooth Filter Implementation
// ======================================================
void AdaptiveFilter::Smooth::init(uint8_t t) {
    taps = std::max<uint8_t>(1, t);
    ring = (float*)calloc(taps, sizeof(float)); // Ensure memory allocation is safe
    idx = 0; sum = 0.0f;
    if (!ring) {
        // Handle allocation failure if necessary
        taps = 0;
        Serial.println("[AdaptiveFilter] Smooth filter memory allocation failed.");
    }
}

void AdaptiveFilter::Smooth::reset() {
    if (!ring) return;
    // CRITICAL FIX: Use std::memset for robustness
    std::memset(ring, 0, sizeof(float) * taps);
    idx = 0; sum = 0.0f;
}

float AdaptiveFilter::Smooth::process(float in) {
    if (!ring) return in;
    sum -= ring[idx];
    ring[idx] = in;
    sum += ring[idx];
    idx = (idx + 1) % taps;
    return sum / (float)taps;
}

void AdaptiveFilter::Smooth::deinit() {
    if (ring) free(ring);
    ring = nullptr; taps = 0; idx = 0; sum = 0.0f;
}

// ======================================================
//    Adaptive Filter Implementation
// ======================================================
AdaptiveFilter::AdaptiveFilter()
    : initialized_(false), active_mode_(FilterMode::BYPASS), target_mode_(FilterMode::BYPASS),
      crossfade_total_frames_(0), crossfade_remaining_(0), crossfade_current_frame_(0),
      scratch_float_buf_(nullptr) {}

AdaptiveFilter::~AdaptiveFilter() {
    if (scratch_float_buf_) free(scratch_float_buf_);
    smooth_filter_.deinit();
}

void AdaptiveFilter::init(const FilterConfig& cfg) {
    cfg_ = cfg;
    initialized_ = true;
    active_mode_ = FilterMode::BYPASS;
    target_mode_ = FilterMode::BYPASS;

    if (scratch_float_buf_) free(scratch_float_buf_);
    // CRITICAL FIX: Ensure scratch buffer allocation matches frame size
    scratch_float_buf_ = (float*)calloc(cfg_.frame_size, sizeof(float));
    if (!scratch_float_buf_) {
        Serial.println("[AdaptiveFilter] Scratch buffer allocation failed!");
        initialized_ = false;
        return;
    }

    smooth_filter_.deinit();
    smooth_filter_.init(cfg_.smooth_taps);

    active_bq_.reset();
    target_bq_.reset();
    rebuildTargetFilter();
    active_bq_ = target_bq_;
}

void AdaptiveFilter::setConfig(const FilterConfig& cfg) {
    cfg_ = cfg;
    rebuildTargetFilter();
}

void AdaptiveFilter::rebuildTargetFilter() {
    // Only rebuild the Biquad filter if the target mode is a Biquad mode.
    switch (target_mode_) {
        case FilterMode::LPF: designBiquadLPF(target_bq_, cfg_.sample_rate, cfg_.lpf_cutoff); break;
        case FilterMode::HPF: designBiquadHPF(target_bq_, cfg_.sample_rate, cfg_.hpf_cutoff); break;
        case FilterMode::BPF: designBiquadBPF(target_bq_, cfg_.sample_rate, cfg_.bpf_center, cfg_.bpf_q); break;
        default: target_bq_.reset(); break;
    }
}

void AdaptiveFilter::setMode(FilterMode mode) {
    if (!initialized_ || mode == active_mode_) return;
    target_mode_ = mode;
    rebuildTargetFilter();
    startCrossfadeTo(mode);
}

void AdaptiveFilter::startCrossfadeTo(FilterMode newMode) {
    crossfade_total_frames_ = std::max<uint8_t>(1, cfg_.crossfade_frames);
    crossfade_remaining_ = crossfade_total_frames_;
    crossfade_current_frame_ = 0;
    // Copy the *current* state of the ACTIVE filter to the TARGET filter
    // so the TARGET starts processing from the ACTIVE filter's last state.
    target_bq_.copyStateFrom(active_bq_); 
}

float AdaptiveFilter::computeFrameEnergy(const int16_t* frame) {
    // Calculate RMS Power (Squared sum)
    double acc = 0.0;
    for (uint32_t i=0; i<cfg_.frame_size; ++i)
        acc += (double)frame[i]*(double)frame[i];
    return (float)(acc / cfg_.frame_size);
}

void AdaptiveFilter::applyNLG(const int16_t* in, int16_t* out, float frameEnergy, const NoiseMetrics& m) {
    // Ensure energy values are positive
    float e = std::max(1e-9f, frameEnergy);
    float nf = std::max(1e-9f, m.noise_floor);

    // Initial gain reduction factor
    float g = 1.0f - (nf / e) * cfg_.nlg_gain_factor;
    
    // Clamp gain
    g = clamp_value(g, cfg_.nlg_min_gain, 1.0f);

    // Further reduction for very low SNR (extra noise suppression)
    if (m.snr_db < 0.0f) {
        // Decrease gain more aggressively when signal is buried in noise (e.g., -60dB range)
        g *= (1.0f - std::min(0.5f, (-m.snr_db / 60.0f)));
    }

    for (uint32_t i=0; i<cfg_.frame_size; ++i) {
        float v = in[i] * g;
        out[i] = float_to_int16(v);
    }
}

void AdaptiveFilter::updateFromNoiseMetrics(const NoiseMetrics& m) {
    last_metrics_ = m;
    if (!initialized_) return;

    // Adaptive Mode Switching Logic
    if (m.snr_db < 8.0f) {
        setMode(FilterMode::ADAPTIVE_NLG);
    }
    else if (m.is_noise_dominant) {
        // Switch to a fixed filter based on noise frequency (spectral centroid)
        if (m.spectral_centroid < 800.0f) setMode(FilterMode::HPF);
        else if (m.spectral_centroid > 3000.0f) setMode(FilterMode::LPF);
        else setMode(FilterMode::BPF);
    } else {
        // If speech is dominant, use a light BPF or SMOOTH
        setMode(FilterMode::SMOOTH); 
    }
}

void AdaptiveFilter::processFrame(const int16_t* input, int16_t* output) {
    if (!initialized_ || !scratch_float_buf_) {
        // Fallback: Copy input to output if initialization failed
        std::memcpy(output, input, cfg_.frame_size * sizeof(int16_t));
        return;
    }

    uint16_t N = cfg_.frame_size;
    // 1. Convert input frame to float for processing
    for (uint16_t i=0; i<N; ++i) scratch_float_buf_[i] = (float)input[i];

    bool crossfading = (crossfade_remaining_ > 0);
    
    // --- Direct Modes (No Biquad involved, faster path) ---
    if (active_mode_ == FilterMode::BYPASS && !crossfading) {
        std::memcpy(output, input, N * sizeof(int16_t));
        return;
    }

    if (active_mode_ == FilterMode::SMOOTH && !crossfading) {
        for (uint16_t i=0;i<N;++i) {
            float s = smooth_filter_.process(scratch_float_buf_[i]);
            output[i] = float_to_int16(s);
        }
        return;
    }

    if (active_mode_ == FilterMode::ADAPTIVE_NLG && !crossfading) {
        // NLG operates directly on the integer frame, no Biquad involved
        float e = computeFrameEnergy(input);
        applyNLG(input, output, e, last_metrics_);
        return;
    }
    
    // --- Biquad Modes (LPF, HPF, BPF) and Crossfading ---

    // Mix factor (0.0=Active filter only, 1.0=Target filter only)
    float mix = crossfading ? (float)crossfade_current_frame_ / crossfade_total_frames_ : 0.0f;

    for (uint16_t i=0;i<N;++i) {
        float x = scratch_float_buf_[i];
        
        // Process sample through both filters
        float y_active = active_bq_.processSample(x);
        float y_target = target_bq_.processSample(x);
        
        // Apply crossfade mix
        float y = crossfading ? (1.0f - mix) * y_active + mix * y_target : y_active;
        
        output[i] = float_to_int16(y);
    }

    // --- Crossfade Management ---
    if (crossfading) {
        crossfade_current_frame_++;
        
        if (crossfade_current_frame_ >= crossfade_total_frames_) {
            // End of crossfade: switch the active filter to the target filter
            active_bq_ = target_bq_;
            active_mode_ = target_mode_;
            crossfade_remaining_ = 0;
            crossfade_current_frame_ = 0;
            // The active filter's state is now implicitly the last processed state from the target
        } else {
            crossfade_remaining_--;
        }
    }
}

// ======================================================
//    Filter Design Helpers
// ======================================================
void AdaptiveFilter::designBiquadLPF(Biquad& bq, float fs, float fc, float Q) {
    float w0 = 2.0f*PI*fc/fs;
    float cosw = cosf(w0);
    float sinw = sinf(w0);
    float alpha = sinw/(2.0f*Q);
    
    // A0 is 1.0 + alpha
    bq.setCoeffs((1.0f-cosw)/2.0f, 1.0f-cosw, (1.0f-cosw)/2.0f, 1.0f+alpha, -2.0f*cosw, 1.0f-alpha);
    bq.reset();
}

void AdaptiveFilter::designBiquadHPF(Biquad& bq, float fs, float fc, float Q) {
    float w0 = 2.0f*PI*fc/fs;
    float cosw = cosf(w0);
    float sinw = sinf(w0);
    float alpha = sinw/(2.0f*Q);
    
    // A0 is 1.0 + alpha
    bq.setCoeffs((1.0f+cosw)/2.0f, -(1.0f+cosw), (1.0f+cosw)/2.0f, 1.0f+alpha, -2.0f*cosw, 1.0f-alpha);
    bq.reset();
}

void AdaptiveFilter::designBiquadBPF(Biquad& bq, float fs, float f0, float Q) {
    float w0 = 2.0f*PI*f0/fs;
    float cosw = cosf(w0);
    float sinw = sinf(w0);
    float alpha = sinw/(2.0f*Q);
    
    // A0 is 1.0 + alpha
    bq.setCoeffs(alpha, 0.0f, -alpha, 1.0f+alpha, -2.0f*cosw, 1.0f-alpha);
    bq.reset();
}

void AdaptiveFilter::printState() {
    Serial.printf("[AdaptiveFilter] mode=%d target=%d SNR=%.1f Centroid=%.1f\n",
                  (int)active_mode_, (int)target_mode_,
                  last_metrics_.snr_db, last_metrics_.spectral_centroid);
}