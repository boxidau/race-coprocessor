#include "Arduino.h"
#include "constants.h"
#include "singletonadc.h"
#include <math.h>
#include <Array.h>

#define NTC_DEFAULT_SAMPLES 100
#define NTC_PRECISION_SAMPLES 100

#define FULL_SCALE_LEAKAGE_CURRENT 0.0307e-6 // Amps @ FS (65535), assume this scales linearly with input voltage
#define FULL_SCALE_VOLTAGE 3.3 // Volts
#define FULL_SCALE_VREF 65516 // FS reading @ 3.3V @ zero leakage

template <int NTC_SAMPLES> class BaseNTC
{
private:
    const uint8_t pin, adcNum;
    const uint32_t pullupResistance;
    const float steinhartA, steinhartB, steinhartC;
    Array<uint16_t, NTC_SAMPLES> samples;
    uint32_t runningSum { 0 };
    uint16_t idx { 0 };

public:
    BaseNTC(
        const uint8_t _pin,
        const uint8_t _adcNum,
        const uint32_t _pullupResistance,
        const float _steinhartA = 1.12865375e-3,
        const float _steinhartB = 2.342041378e-4,
        const float _steinhartC = 8.737724626e-8
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
        pinMode(pin, INPUT_DISABLE);
    }

    uint16_t acquireAndDiscardSample() {
        return SingletonADC::getADC()->analogRead(pin, adcNum);
    }
    
    void loop() {
        uint16_t curValue = SingletonADC::getADC()->analogRead(pin, adcNum);
        if (!samples.full()) {
            samples.push_back(curValue);
        } else {
            runningSum -= samples[idx % NTC_SAMPLES];
            samples[idx % NTC_SAMPLES] = curValue;
        }
        idx++;
        runningSum += curValue;
    }

    float temperature() {
        return temperatureFor(adc());        
    }

    uint16_t adc() {
        return !samples.empty() ? round((float) runningSum / samples.size()) : 0;
    }

    uint16_t adcCalculateAverage() {
        if (samples.empty()) {
            return 0;
        }

        uint32_t runningSum2 = 0;
        for (size_t i = 0; i < samples.size(); i++) {
            runningSum2 += samples[i];
        }
        return round((float) runningSum2 / samples.size());
    }

    uint16_t latest() {
        return !samples.empty() ? samples[(idx - 1) % NTC_SAMPLES] : 0;
    };

    float temperatureFor(uint16_t sample) {
        float ntcResistanceApprox = pullupResistance / (((float) ADC_MAX / sample) - 1);
        // double fullScaleLeakageOffset = 1 / (1 / ntcResistanceApprox + 1 / pullupResistance) * FULL_SCALE_LEAKAGE_CURRENT / FULL_SCALE_VOLTAGE;
        // double offsetAdjustedVal = val + fullScaleLeakageOffset * val;
        // double ntcResistance = pullupResistance / ((FULL_SCALE_VREF / offsetAdjustedVal) - 1);
        float lnR = log(ntcResistanceApprox);
        // expand out lnR * lnR * lnR, much faster than pow()
        return 1 / (steinhartA + (steinhartB * lnR) + (steinhartC * lnR * lnR * lnR)) - 273.15;
    }
};

typedef BaseNTC<NTC_DEFAULT_SAMPLES> NTC;
typedef BaseNTC<NTC_PRECISION_SAMPLES> PrecisionNTC;
