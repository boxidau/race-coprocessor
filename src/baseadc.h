#pragma once

#include <Array.h>
#include "singletonadc.h"
#include <DebugLog.h>

template <int BASE_ADC_SAMPLES> class BaseADC {
public:
    BaseADC(const uint8_t _pin, const uint8_t _adcNum)
        : pin { _pin }
        , adcNum { _adcNum }
    {
    };
    
    void setup() {
        pinMode(pin, INPUT_DISABLE);
    }

    void loop() {
        uint16_t curValue = SingletonADC::getADC()->analogRead(pin, adcNum);
        if (!samples.full()) {
            samples.push_back(curValue);
        } else {
            runningSum -= samples[idx];
            samples[idx] = curValue;
        }
        idx++;
        if (idx == BASE_ADC_SAMPLES) {
            idx = 0;
        }
        runningSum += curValue;

    }

    uint16_t adc() {
        return !samples.empty() ? round((float) runningSum / samples.size()) : 0;
    }

    uint16_t latest() {
        return !samples.empty() ? samples[idx > 0 ? idx - 1 : samples.size() - 1] : 0;
    };

protected:
    const uint8_t pin;
    const uint8_t adcNum;
    Array<uint16_t, BASE_ADC_SAMPLES> samples;
    uint32_t idx = { 0 };
    uint32_t runningSum { 0 };
};
