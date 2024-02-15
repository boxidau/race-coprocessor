#include "calibratedadc.h"

void CalibratedADC::loop() {
    uint16_t curValue = SingletonADC::getADC()->analogRead(pin, adcNum);
    if (!samples.full()) {
        samples.push_back(curValue);
    } else {
        runningSum -= samples[idx % ADC_SAMPLES];
        samples[idx % ADC_SAMPLES] = curValue;
    }
    idx++;
    runningSum += curValue;
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
    return !samples.empty() ? round((double) runningSum / samples.size()) : 0;
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