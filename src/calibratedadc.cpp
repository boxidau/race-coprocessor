#include "calibratedadc.h"

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

uint16_t CalibratedADC::calibratedValue() {
    uint16_t _adc = this->adc();
    if (constrainCalibration) {
        _adc = constrain(_adc, calibrationLowADC, calibrationHighADC);
    }

    return map(
        _adc,
        calibrationLowADC,
        calibrationHighADC,
        calibrationLowValue,
        calibrationHighValue
    );
}
