#pragma once

#include "Arduino.h"
#include <DebugLog.h>
#include <Array.h>
#include "constants.h"
#include "singletonadc.h"

#define SWITCH_ADC_SAMPLES 20

enum class CoolerSwitchPosition {
    UNKNOWN        = -1,
    RESET          = 0,
    PRECHILL       = 1,
    PUMP_LOW       = 2,
    PUMP_MEDIUM    = 3,
    PUMP_HIGH      = 4
};

class SwitchADC
{
private:
    const uint8_t pin;
    uint8_t idx = { 0 };
    Array<uint16_t, SWITCH_ADC_SAMPLES> samples;
    uint8_t adcNum;

public:
    SwitchADC(const uint8_t _pin, const uint8_t _adcNum) : pin { _pin }, adcNum { _adcNum } {
    };
    
    void setup() {
        pinMode(pin, INPUT_DISABLE);
    }

    void loop();
    uint16_t adc();
    CoolerSwitchPosition position();
};