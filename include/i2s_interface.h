#ifndef I2S_INTERFACE_H
#define I2S_INTERFACE_H

#include <Arduino.h>

class I2SInterface {
public:
    // Initialize I2S hardware
    static void init();

    // Read audio samples into buffer, returning the number of samples read.
    // Converts 32-bit raw data to 16-bit PCM.
    static int readAudio(int16_t *samples, size_t num_samples);

    // Optional test function for debugging mic input
    static void testMic();
};

#endif
