#include "calibratedadc.h"

void CalibratedADC::setCalibration(
    uint16_t lowADC,
    int32_t lowValue,
    uint16_t highADC,
    int32_t highValue,
    bool constrain
) {
    calibrationLowADC = lowADC;
    calibrationLowValue = lowValue;
    calibrationHighADC = highADC;
    calibrationHighValue = highValue;
    constrainCalibration = constrain;
}

int32_t CalibratedADC::calibrate(uint16_t value) {
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

int32_t CalibratedADC::calibratedValue() {
    return calibrate(this->adc());
}

int32_t CalibratedADC::calibratedLatestValue() {
    return calibrate(this->latest());
}
