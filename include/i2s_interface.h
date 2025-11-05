#ifndef I2S_INTERFACE_H
#define I2S_INTERFACE_H

#include <stdint.h>
#include "driver/i2s.h"

// I2S Configuration
#define I2S_WS      25   // LRCL (Word Select)
#define I2S_SD      33   // DOUT (Data In)
#define I2S_SCK     14   // BCLK (Bit Clock)
#define I2S_PORT    I2S_NUM_0
#define SAMPLE_RATE 16000

class I2SInterface {
public:
    static bool init();
    static size_t readSamples(int16_t* buffer, size_t count);
    static void cleanup();
    
private:
    static bool initialized_;
};

#endif