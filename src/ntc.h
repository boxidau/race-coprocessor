#include "Arduino.h"
#include "constants.h"
#include "singletonadc.h"
#include <math.h>
#include <DebugLog.h>

#define NTC_SAMPLES 100

class NTC
{
private:
    const uint8_t pin, adcNum;
    const double pullupResistance, steinhartA, steinhartB, steinhartC;
    uint16_t samples[NTC_SAMPLES];
    uint8_t idx { 0 };

public:
    NTC(
        const uint8_t _pin,
        const uint8_t _adcNum,
        const double _pullupResistance,
        const double _steinhartA = 0.00112865375,
        const double _steinhartB = 0.0002342041378,
        const double _steinhartC = 0.00000008737724626
    )
        : pin { _pin }
        , adcNum { _adcNum }
        , pullupResistance { _pullupResistance }
        , steinhartA { _steinhartA }
        , steinhartB { _steinhartB }
        , steinhartC { _steinhartC }
    {
    };
    
    void setup() {
        pinMode(pin, INPUT);
    }
    void loop();
    double temperature();
    uint16_t adc();

    double temperatureFor(uint16_t);

    uint16_t min();
    uint16_t max();
    uint16_t stdev();
    double temperatureStdev();
};