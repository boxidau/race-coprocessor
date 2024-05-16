#pragma once

#include "baseadc.h"

#define ADC_SAMPLES 100

class CalibratedADC : public BaseADC<ADC_SAMPLES>
{
private:
    uint16_t calibrationLowADC, calibrationHighADC;
    int32_t calibrationLowValue, calibrationHighValue;
    bool constrainCalibration { false };

    int32_t calibrate(uint16_t value);

public:
    CalibratedADC(const uint8_t _pin, const uint8_t _adcNum) : 
        BaseADC(_pin, _adcNum)
    {
    };

    void setCalibration(
        uint16_t lowADC,
        int32_t lowValue,
        uint16_t highADC,
        int32_t highValue,
        bool constrain
    );

    int32_t calibratedValue();
    int32_t calibratedLatestValue();
};