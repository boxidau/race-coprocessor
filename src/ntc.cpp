#include "ntc.h"

void NTC::loop() {
    samples[idx++ % NTC_SAMPLES] = analogRead(pin);
}

double NTC::temperature() {
    double ntcResistance = pullupResistance / ((ADC_MAX / (double)adc()) - 1);
    double lnR = log(ntcResistance);
    return (
        1 / (
            steinhartA + (steinhartB * lnR) + (steinhartC * pow(lnR, 3))
        )
    ) - 273.15;
}

uint16_t NTC::adc() {
    uint sum = 0;
    for (uint i = 0; i < NTC_SAMPLES; i++) {
        sum += samples[i];
    }
    return (sum / NTC_SAMPLES);
}