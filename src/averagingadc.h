#include "Arduino.h"
#include "constants.h"
#include <DebugLog.h>

#define ADC_SAMPLES 100

class AveragingADC
{
private:
    const uint8_t pin;
    uint8_t idx = { 0 };
    uint16_t samples[ADC_SAMPLES];

public:
    AveragingADC(const uint8_t _pin) : pin { _pin } {};
    
    void setup() {
        pinMode(pin, INPUT);
    }
    void loop();
    uint16_t adc();
    uint16_t minV();
    uint16_t maxV();
};