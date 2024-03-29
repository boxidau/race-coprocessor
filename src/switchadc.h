#pragma once

#include "Arduino.h"
#include "constants.h"
#include <DebugLog.h>
#include "singletonadc.h"

#define SWITCH_ADC_SAMPLES 10

enum class CoolerSwitchPosition {
    UNKNOWN        = 0,
    RESET          = 1,
    PRECHILL       = 2,
    PUMP_LOW       = 3,
    PUMP_MEDIUM    = 4,
    PUMP_HIGH      = 5
};

class SwitchADC
{
private:
    const uint8_t pin;
    uint8_t idx = { 0 };
    uint16_t samples[SWITCH_ADC_SAMPLES];
    uint8_t adcNum;

public:
    SwitchADC(const uint8_t _pin, const uint8_t _adcNum) : pin { _pin }, adcNum { _adcNum } {
    };
    
    void setup() {
        pinMode(pin, INPUT);
    }

    void loop();
    uint16_t adc();
    CoolerSwitchPosition position();
};