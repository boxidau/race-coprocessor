#include "calibratedadc.h"

void CalibratedADC::loop() {
    samples[idx % ADC_SAMPLES] = SingletonADC::getADC()->analogRead(pin, adcNum);
    idx++;
    if (idx == ADC_SAMPLES) {
        bufferInitialFill = true;
    }
}

void CalibratedADC::setCalibration(
    uint16_t lowADC,
    uint16_t lowValue,
    uint16_t highADC,
    uint16_t highValue,
    bool constrain
) {
    calibrationLowADC = lowADC;
    calibrationLowValue = lowValue;
    calibrationHighADC = highADC;
    calibrationHighValue = highValue;
    constrainCalibration = constrain;
}

const uint16_t CalibratedADC::adc() {
    if (!bufferInitialFill) {
        return samples[(idx - 1) % ADC_SAMPLES];
    }
    uint32_t sum = 0;
    for (uint i = 0; i < ADC_SAMPLES; i++) {
        sum += samples[i];
        // LOG_INFO(samples[i]);
    }
    // LOG_INFO("END");
    return uint16_t(sum / ADC_SAMPLES);
}

const uint16_t CalibratedADC::minV() {
    uint16_t min = 65535;
    for (uint i = 0; i < ADC_SAMPLES; i++) {
        if (samples[i] < min) min = samples[i];
    }
    return min;
}

const uint16_t CalibratedADC::maxV() {
    uint16_t max = 0;
    for (uint i = 0; i < ADC_SAMPLES; i++) {
        if (samples[i] > max) max = samples[i];
    }
    return max;
}

const uint16_t CalibratedADC::calibratedValue() {
    uint16_t _adc = adc();
    if (constrainCalibration) _adc = constrain(
        adc(), calibrationLowADC, calibrationHighADC
    );
        
    return map(
        _adc,
        calibrationLowADC,
        calibrationHighADC,
        calibrationLowValue,
        calibrationHighValue
    );
};