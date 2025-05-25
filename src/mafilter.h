#pragma once

#include <Array.h>
#include <type_traits> // Required for std::is_integral

template <typename SAMPLE_TYPE, typename INTEGER_TYPE, int SAMPLES, int PRECISION> class MAFilter {
public:
    void push(SAMPLE_TYPE curValue) {
        INTEGER_TYPE roundedValue = round(curValue * PRECISION);
        if (!samples.full()) {
            samples.push_back(roundedValue);
        } else {
            runningSum -= samples[idx];
            samples[idx] = roundedValue;
        }
        idx++;
        if (idx == SAMPLES) {
            idx = 0;
        }
        runningSum += roundedValue;
    }

    SAMPLE_TYPE filteredValue() {
        if constexpr (std::is_integral<SAMPLE_TYPE>::value) {
            return !samples.empty() ? round((float) runningSum / samples.size() / PRECISION) : 0;
        }

        return !samples.empty() ? (SAMPLE_TYPE) runningSum / samples.size() / PRECISION : 0;
    }

    SAMPLE_TYPE latest() {
        return !samples.empty() ? (SAMPLE_TYPE) samples[idx > 0 ? idx - 1 : samples.size() - 1] / PRECISION : 0;
    };

protected:
    Array<INTEGER_TYPE, SAMPLES> samples;
    uint32_t idx = { 0 };
    int32_t runningSum { 0 };
};
