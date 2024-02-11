#include "ntc.h"
#include <ADC.h>
#include <QuickMedianLib.h>

void NTC::loop() {
    uint16_t curValue = !differentialPin ? 
        SingletonADC::getADC()->analogRead(pin, adcNum) :
        SingletonADC::getADC()->analogReadDifferential(pin, differentialPin, adcNum);

    samples[idx % NTC_SAMPLES] = curValue;
    runningSum += curValue;
    idx++;
    runningSum -= samples[idx % NTC_SAMPLES];
    filteredSamples.clear();
    filteredSum = 0;
}

double NTC::temperature() {
    if (differentialPin) {
        return 0;
    }

    return temperatureFor(((double) runningSum) / NTC_SAMPLES);
}

double NTC::temperatureFor(double val) {
    // note: this may be wrong because noise > 65535 is cut off. look at time domain signal
    double ntcResistanceApprox = pullupResistance / ((ADC_MAX / val) - 1);
    // double fullScaleLeakageOffset = 1 / (1 / ntcResistanceApprox + 1 / pullupResistance) * FULL_SCALE_LEAKAGE_CURRENT / FULL_SCALE_VOLTAGE;
    // double offsetAdjustedVal = val + fullScaleLeakageOffset * val;
    // double ntcResistance = pullupResistance / ((FULL_SCALE_VREF / offsetAdjustedVal) - 1);
    double lnR = log(ntcResistanceApprox);
    return (
        1 / (
            steinhartA + (steinhartB * lnR) + (steinhartC * pow(lnR, 3))
        )
    ) - 273.15;
}

uint16_t NTC::latest() {
    return samples[(idx - 1) % NTC_SAMPLES];
}

double NTC::adc() {
    return ((double) runningSum) / NTC_SAMPLES;
}

uint16_t NTC::min() {
    uint16_t min = ADC_MAX;
    for (uint i = 0; i < NTC_SAMPLES; i++) {
        if (samples[i] < min) min = samples[i];
    }
    return min;
}

uint16_t NTC::max() {
    uint16_t max = 0;
    for (uint i = 0; i < NTC_SAMPLES; i++) {
        if (samples[i] > max) max = samples[i];
    }
    return max;
}

uint16_t NTC::median() {
    return QuickMedian<uint16_t>::GetMedian(samples, NTC_SAMPLES);
}

double NTC::stdev() {
    double var = 0;
    double avg = ((double) runningSum) / NTC_SAMPLES;
    for (uint i = 0; i < NTC_SAMPLES; i++) {
        var += pow(double(samples[i]) - avg, 2);
    }
    return sqrt(var / NTC_SAMPLES);
}

double NTC::temperatureStdev() {
    if (differentialPin) {
        return 0;
    }

    double var = 0;
    double avg = temperature();
    for (uint i = 0; i < NTC_SAMPLES; i++) {
        var += pow(temperatureFor(samples[i]) - avg, 2);
    }
    return sqrt(var / NTC_SAMPLES);
}

void NTC::memoizeFilteredSamples() {
    if (filteredSamples.size()) {
        // already memoized
        return;
    }

    double threshold = stdev() * 2; // 2 stdev ~= 95% of samples
    if (threshold < 2) {
        // set to a reasonable value so we don't discard all samples
        threshold = 2;
    }

    double average = ((double) runningSum) / NTC_SAMPLES;
    filteredSum = 0;
    for (uint i = 0; i < NTC_SAMPLES; i++) {
        if (abs(samples[i] - average) <= threshold) {
            filteredSamples.push_back(samples[i]);
            filteredSum += samples[i];
        }
    }

    if (filteredSamples.size() < 0.8 * NTC_SAMPLES) {
        LOG_WARN("Discarded > 20\% of samples, returning original array");
        filteredSamples.clear();
        for (uint i = 0; i < NTC_SAMPLES; i++) {
            filteredSamples.push_back(samples[i]);
        }
    }
}

double NTC::averageWithoutOutliers() {
    memoizeFilteredSamples();
    if (filteredSamples.size() == 0) {
        return 0;
    }

    return (double) filteredSum / filteredSamples.size();
}

double NTC::stdevWithoutOutliers() {
    memoizeFilteredSamples();
    if (filteredSamples.size() == 0) {
        return 0;
    }

    double avg = (double) filteredSum / filteredSamples.size();
    double var = 0;
    for (uint i = 0; i < filteredSamples.size(); i++) {
        var += pow(double(filteredSamples[i]) - avg, 2);
    }
    return sqrt(var / filteredSamples.size());
}

double NTC::temperatureStdevWithoutOutliers() {
    if (differentialPin) {
        return 0;
    }

    memoizeFilteredSamples();
    if (filteredSamples.size() == 0) {
        return 0;
    }

    double var = 0;
    double avg = temperatureFor((double) filteredSum / filteredSamples.size());
    for (uint i = 0; i < filteredSamples.size(); i++) {
        var += pow(temperatureFor(filteredSamples[i]) - avg, 2);
    }
    return sqrt(var / filteredSamples.size());
}
