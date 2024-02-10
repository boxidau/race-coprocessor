#include "Arduino.h"
#include "constants.h"
#include "singletonadc.h"
#include <math.h>
#include <DebugLog.h>
#include <Array.h>

#define NTC_SAMPLES 101 // odd number so median is an integer
#define FULL_SCALE_LEAKAGE_CURRENT 0.0307e-6 // Amps @ FS (65535), assume this scales linearly with input voltage
#define FULL_SCALE_VOLTAGE 3.3 // Volts
#define FULL_SCALE_VREF 65516 // FS reading @ 3.3V @ zero leakage

class NTC
{
private:
    const uint8_t pin, adcNum;
    const double pullupResistance;
    const uint8_t differentialPin;
    double steinhartA, steinhartB, steinhartC;
    uint16_t samples[NTC_SAMPLES];
    uint32_t runningSum { 0 };
    uint8_t idx { 0 };
    Array<uint16_t, NTC_SAMPLES> filteredSamples;
    uint32_t filteredSum { 0 };

public:
    NTC(
        const uint8_t _pin,
        const uint8_t _adcNum,
        const double _pullupResistance,
        const uint8_t _differentialPin = 0,
        const double _steinhartA = 0.00112865375,
        const double _steinhartB = 0.0002342041378,
        const double _steinhartC = 0.00000008737724626
    )
        : pin { _pin }
        , adcNum { _adcNum }
        , pullupResistance { _pullupResistance }
        , differentialPin { differentialPin }
        , steinhartA { _steinhartA }
        , steinhartB { _steinhartB }
        , steinhartC { _steinhartC }
    {
    };
    
    void setup() {
        pinMode(pin, INPUT);
        if (differentialPin) {
            pinMode(differentialPin, INPUT);
        }
    }
    void loop();
    double temperature();
    double adc();
    uint16_t latest();

    double temperatureFor(double temp);

    uint16_t min();
    uint16_t max();
    uint16_t median();
    double stdev();
    double temperatureStdev();

    double averageWithoutOutliers();
    double stdevWithoutOutliers();
    double temperatureStdevWithoutOutliers();

private:
    void memoizeFilteredSamples();
};