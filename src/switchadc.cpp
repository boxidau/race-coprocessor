#include "calibratedadc.h"
#include <ADC.h>

void SwitchADC::loop() {
    samples[idx % ADC_SAMPLES] = ADC().analogRead(pin, adc);
    idx++;
}

CoolerSwitchPosition _getSwitchPosition(uint16_t switchADCValue)
{
    // UNKNOWN means switch is open, likely in process of switching positions
    if (switchADCValue < 15000) return CoolerSwitchPosition::RESET;
    if (switchADCValue < 26000) return CoolerSwitchPosition::PRECHILL;
    if (switchADCValue < 38000) return CoolerSwitchPosition::PUMP_LOW;
    if (switchADCValue < 49000) return CoolerSwitchPosition::PUMP_MEDIUM;
    if (switchADCValue < 60000) return CoolerSwitchPosition::PUMP_HIGH;
    return CoolerSwitchPosition::UNKNOWN;
}

CoolerSwitchPosition SwitchADC::position()
{
    // check every sample agrees, if not then the switch is likely in a bounce state
    for (uint i = 1; i < ADC_SAMPLES; i++) {
        if (_getSwitchPosition(samples[i]) != _getSwitchPosition(samples[i-1])) {
            LOG_INFO("Switch position samples disagree, returning UNKNOWN position", samples[i], samples[i-1]);
            return CoolerSwitchPosition::UNKNOWN;
        }
    }
    return _getSwitchPosition(samples[0]);
}

uint16_t SwitchADC::adc() {
    uint32_t sum = 0;
    for (uint i = 0; i < ADC_SAMPLES; i++) {
        sum += samples[i];
    }
    return uint16_t(sum / ADC_SAMPLES);
}
