#pragma once

#include "Arduino.h"
#include "constants.h"
#include <DebugLog.h>
#include "singletonadc.h"
#include <Array.h>

#define ADC_SAMPLES 100

class CalibratedADC
{
private:
    const uint8_t pin;
    uint8_t idx = { 0 };
    Array<uint16_t, ADC_SAMPLES> samples;
    uint32_t runningSum { 0 };
    uint16_t calibrationLowADC, calibrationLowValue;
    uint16_t calibrationHighADC, calibrationHighValue;
    bool constrainCalibration { false };
    const uint8_t adcNum;

public:
    CalibratedADC(const uint8_t _pin, const uint8_t _adcNum) : pin { _pin }, adcNum { _adcNum } {};
    
    void setup() {
        pinMode(pin, INPUT);

    }
    void setCalibration(
        uint16_t lowADC,
        uint16_t lowValue,
        uint16_t highADC,
        uint16_t highValue,
        bool constrain
    );
    void loop();
    const uint16_t adc();
    const uint16_t calibratedValue();
};