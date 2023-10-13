#include "averagingadc.h"

void AveragingADC::loop() {
    samples[idx % ADC_SAMPLES] = analogRead(pin);
    idx++;
}

uint16_t AveragingADC::adc() {
    // return analogRead(pin);
    uint32_t sum = 0;
    for (uint i = 0; i < ADC_SAMPLES; i++) {
        sum += samples[i];
        // LOG_INFO(samples[i]);
    }
    // LOG_INFO("END");
    return uint16_t(sum / ADC_SAMPLES);
}