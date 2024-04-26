#pragma once

#include "baseadc.h"

#define ADC_SAMPLES 100

class CalibratedADC : public BaseADC<ADC_SAMPLES>
{
private:
    uint16_t calibrationLowADC, calibrationLowValue;
    uint16_t calibrationHighADC, calibrationHighValue;
    bool constrainCalibration { false };

    uint16_t calibrate(uint16_t value);

public:
    CalibratedADC(const uint8_t _pin, const uint8_t _adcNum) : 
        BaseADC(_pin, _adcNum)
    {
    };

    void setCalibration(
        uint16_t lowADC,
        uint16_t lowValue,
        uint16_t highADC,
        uint16_t highValue,
        bool constrain
    );

    uint16_t calibratedValue();
    uint16_t calibratedLatestValue();
};