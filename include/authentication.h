
#pragma once
#include <Arduino.h>
#include <vector>

// NOTE: MFCC features are replaced by a generic feature vector of size 2 (Mean, Energy) 
// until a full MFCC library is integrated.
struct VoiceTemplate {
    String username;
    std::vector<float> features; // Now generic features, not just MFCC
};

// Global authentication functions
void auth_init();
bool enrollUser(const String &username, const std::vector<float> &features);
bool authenticateUser(const std::vector<float> &features, String &matchedUser);
std::vector<float> extractFeatures(const int16_t *audioData, size_t length);

float cosineSimilarity(const std::vector<float> &a, const std::vector<float> &b);

