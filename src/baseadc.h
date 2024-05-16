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

    uint16_t min() {
        uint16_t minVal = UINT16_MAX;
        for (size_t i = 0; i < samples.size(); i++) {
            minVal = ::min(minVal, samples[i]);
        }
        return minVal;
    }

    uint16_t max() {
        uint16_t maxVal = 0;
        for (size_t i = 0; i < samples.size(); i++) {
            maxVal = ::max(maxVal, samples[i]);
        }
        return maxVal;
    }

    float stdev() {
        if (samples.empty()) {
            return 0;
        }

        uint32_t var = 0;
        // multiply by 100 to get additional resolution for small stdev values
        uint32_t avg = runningSum * 100 / samples.size();
        for (size_t i = 0; i < samples.size(); i++) {
            int32_t diff = (uint32_t) samples[i] * 100 - avg; 
            var += diff * diff;
        }
        return sqrtf((float) var / (samples.size() * 100 * 100));
    }

protected:
    const uint8_t pin;
    const uint8_t adcNum;
    Array<uint16_t, BASE_ADC_SAMPLES> samples;
    uint32_t idx = { 0 };
    uint32_t runningSum { 0 };
};
