#include "ntc.h"

void NTC::loop() {
    uint16_t curValue = analogRead(pin);
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

double NTC::minV() {
    uint16_t min = 65535;
    for (uint i = 0; i < NTC_SAMPLES; i++) {
        if (samples[i] < min) min = samples[i];
    }
    return temperatureFor(min);
}

double NTC::maxV() {
    uint16_t max = 0;
    for (uint i = 0; i < NTC_SAMPLES; i++) {
        if (samples[i] > max) max = samples[i];
    }
    return temperatureFor(max);
}