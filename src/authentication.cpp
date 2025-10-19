#include "authentication.h"
#include <cmath>
#include <numeric> // For std::accumulate (optional, but good practice)

// Authentication Database
static std::vector<VoiceTemplate> userDB;

void auth_init() {
    // CRITICAL FIX: Removed duplicate Serial.begin(), which should be in setup()
    Serial.println("[AUTH] Authentication Engine initialized");
}

std::vector<float> extractFeatures(const int16_t *audioData, size_t length) {
    // --- CRITICAL FIX: Replaced flawed MFCC placeholder with simple Mean/Energy ---
    // NOTE: For true voice authentication, this function must be replaced by a full MFCC
    // or similar feature extraction library using the input audioData.

    if (length == 0) {
        return {0.0f, 0.0f}; // Return zero features if no data
    }
    
    // 1. Calculate Mean (DC offset)
    long long sum = 0;
    long long energy_sq = 0;

    for (size_t i = 0; i < length; i++) {
        sum += audioData[i];
        // Use long long for intermediate squared sum to prevent overflow (16-bit max is 32767)
        energy_sq += (long long)audioData[i] * audioData[i]; 
    }

    float mean = (float)sum / length;
    // 2. Calculate RMS Energy
    float avgEnergy = sqrtf((float)energy_sq / length);

    // Return two features: [Mean, RMS Energy]
    return {mean, avgEnergy};
}

bool enrollUser(const String &username, const std::vector<float> &features) {
    for (auto &u : userDB) {
        // If username exists, check if features are similar before overwriting/rejecting
        if (u.username == username) {
            Serial.println("[AUTH] User already enrolled, rejecting new enrollment.");
            return false;
        }
    }

    // Use u.features for generic name consistent with the struct update
    userDB.push_back({username, features}); 
    Serial.println("[AUTH] Enrolled user: " + username + " (Features size: " + String(features.size()) + ")");
    return true;
}

bool authenticateUser(const std::vector<float> &features, String &matchedUser) {
    if (userDB.empty()) {
        Serial.println("[AUTH] Authentication failed. No users enrolled.");
        return false;
    }

    float bestScore = -1.0;
    String bestUser = "";
    
    // Threshold for similarity (0.85 is too high for simple Mean/Energy, 
    // but a starting point for complex features)
    static constexpr float AUTH_THRESHOLD = 0.80f; 

    for (auto &u : userDB) {
        // Compare current input features against stored template features
        float sim = cosineSimilarity(u.features, features); 
        
        if (sim > bestScore) {
            bestScore = sim;
            bestUser = u.username;
        }
    }

    if (bestScore >= AUTH_THRESHOLD) {
        matchedUser = bestUser;
        Serial.println("[AUTH] Match found: " + matchedUser + " (score=" + String(bestScore, 4) + ")");
        return true;
    }

    Serial.println("[AUTH] Authentication failed. Best score=" + String(bestScore, 4) + ". Threshold=" + String(AUTH_THRESHOLD));
    return false;
}

float cosineSimilarity(const std::vector<float> &a, const std::vector<float> &b) {
    if (a.size() != b.size() || a.empty()) return 0.0f;
    
    float dot = 0.0f, magA = 0.0f, magB = 0.0f;
    for (size_t i = 0; i < a.size(); i++) {
        dot += a[i] * b[i];
        magA += a[i] * a[i];
        magB += b[i] * b[i];
    }
    
    float denominator = sqrtf(magA) * sqrtf(magB);
    
    // Add small epsilon (1e-6) to denominator to prevent division by zero for silence/zero-vectors
    if (denominator < 1e-6f) return 0.0f;
    
    return dot / denominator;
}