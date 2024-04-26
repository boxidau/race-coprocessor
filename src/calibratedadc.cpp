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

uint16_t CalibratedADC::calibrate(uint16_t value) {
    if (constrainCalibration) {
        value = constrain(value, calibrationLowADC, calibrationHighADC);
    }

    return map(
        value,
        calibrationLowADC,
        calibrationHighADC,
        calibrationLowValue,
        calibrationHighValue
    );
}

uint16_t CalibratedADC::calibratedValue() {
    return calibrate(this->adc());
}

uint16_t CalibratedADC::calibratedLatestValue() {
    return calibrate(this->latest());
}
