#pragma once

#include "baseadc.h"

#define SWITCH_ADC_SAMPLES 20

enum class CoolerSwitchPosition {
    UNKNOWN        = -1,
    RESET          = 0,
    PRECHILL       = 1,
    PUMP_LOW       = 2,
    PUMP_MEDIUM    = 3,
    PUMP_HIGH      = 4
};

class SwitchADC : public BaseADC<SWITCH_ADC_SAMPLES>
{
public:
    SwitchADC(const uint8_t _pin, const uint8_t _adcNum)
        : BaseADC(_pin, _adcNum)
    {
    };

    CoolerSwitchPosition position();
};