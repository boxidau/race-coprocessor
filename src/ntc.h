#include "Arduino.h"
#include "constants.h"
#include <math.h>
#include <DebugLog.h>

class NTC
{
private:
    const uint8_t pin;
    const double steinhartA, steinhartB, steinhartC, pullupResistance;
public:
    NTC(
        const uint8_t _pin,
        const double _steinhartA = 0.00112865375,
        const double _steinhartB = 0.0002342041378,
        const double _steinhartC = 0.00000008737724626,
        const double _pullupResistance = 2400
    )
        : pin { _pin }
        , steinhartA { _steinhartA }
        , steinhartB { _steinhartB }
        , steinhartC { _steinhartC }
        , pullupResistance { _pullupResistance }
    {
        
    };
    
    void setup() {
        pinMode(pin, INPUT);
    }
    double temperature();
    uint16_t adc();
};