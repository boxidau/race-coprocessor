#include "ntc.h"
#include <ADC.h>

void NTC::loop() {
    uint16_t curValue = ADC().analogRead(pin, adcNum);
    samples[idx++ % NTC_SAMPLES] = curValue;
    rollingSum += curValue;
    rollingSum -= samples[idx % NTC_SAMPLES];
}

double NTC::temperature() {
    return temperatureFor(adc());
}

double NTC::temperatureFor(uint16_t val) {
    double ntcResistance = pullupResistance / ((ADC_MAX / (double)val) - 1);
    double lnR = log(ntcResistance);
    return (
        1 / (
            steinhartA + (steinhartB * lnR) + (steinhartC * pow(lnR, 3))
        )
    ) - 273.15;
}

uint16_t NTC::adc() {
    return rollingSum / NTC_SAMPLES;
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

uint16_t NTC::stdev() {
    double var = 0;
    double avg = double(rollingSum) / NTC_SAMPLES;
    for (uint i = 0; i < NTC_SAMPLES; i++) {
        var += pow(double(samples[i]) - avg, 2);
    }
    return uint16_t(sqrt(var / NTC_SAMPLES));
}

double NTC::temperatureStdev() {
    double var = 0;
    double avg = temperature();
    for (uint i = 0; i < NTC_SAMPLES; i++) {
        var += pow(temperatureFor(samples[i]) - avg, 2);
    }
    return sqrt(var / NTC_SAMPLES);
}