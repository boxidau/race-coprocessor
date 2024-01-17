#pragma once

#include "Arduino.h"
#include "constants.h"
#include <DebugLog.h>
#include <ADC.h>

#define ADC_SAMPLES 100

class CalibratedADC
{
private:
    const uint8_t pin;
    uint8_t idx = { 0 };
    uint16_t samples[ADC_SAMPLES];
    uint16_t calibrationLowADC, calibrationLowValue;
    uint16_t calibrationHighADC, calibrationHighValue;
    bool constrainCalibration { false };
    ADC_Module* adcObj;

public:
    CalibratedADC(const uint8_t _pin, const uint8_t _adcNum) : pin { _pin }, adcObj { ADC().adc[_adcNum] } {
    };
    
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
    const uint16_t minV();
    const uint16_t maxV();
    const uint16_t calibratedValue();
};