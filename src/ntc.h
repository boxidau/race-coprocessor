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
    const double pullupResistance;
    double steinhartA, steinhartB, steinhartC;
    Array<uint16_t, NTC_SAMPLES> samples;
    uint32_t runningSum { 0 };
    uint16_t idx { 0 };

public:
    BaseNTC(
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

    double temperature() {
        return temperatureFor(adc());        
    }

    uint16_t adc() {
        return !samples.empty() ? round((double) runningSum / samples.size()) : 0;
    }

    uint16_t latest() {
        return !samples.empty() ? samples[(idx - 1) % NTC_SAMPLES] : 0;
    };

    double temperatureFor(uint16_t sample) {
        double ntcResistanceApprox = pullupResistance / (((double) ADC_MAX / sample) - 1);
        // double fullScaleLeakageOffset = 1 / (1 / ntcResistanceApprox + 1 / pullupResistance) * FULL_SCALE_LEAKAGE_CURRENT / FULL_SCALE_VOLTAGE;
        // double offsetAdjustedVal = val + fullScaleLeakageOffset * val;
        // double ntcResistance = pullupResistance / ((FULL_SCALE_VREF / offsetAdjustedVal) - 1);
        double lnR = log(ntcResistanceApprox);
        return  1 / (steinhartA + (steinhartB * lnR) + (steinhartC * pow(lnR, 3))) - 273.15;
    }
};

typedef BaseNTC<NTC_DEFAULT_SAMPLES> NTC;
typedef BaseNTC<NTC_PRECISION_SAMPLES> PrecisionNTC;
