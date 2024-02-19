#include "switchadc.h"
#include <ADC.h>

unsigned long start;

void SwitchADC::loop() {
    uint16_t curValue = SingletonADC::getADC()->analogRead(pin, adcNum);
    if (!samples.full()) {
        samples.push_back(curValue);
    } else {
        samples[idx % samples.max_size()] = curValue;
    }
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
    if (samples.size() < samples.max_size()) {
        return CoolerSwitchPosition::UNKNOWN;
    }

    // check every sample agrees, if not then the switch is likely in a bounce state
    for (size_t i = 1; i < samples.size(); i++) {
        if (_getSwitchPosition(samples[i]) != _getSwitchPosition(samples[i - 1])) {
            LOG_INFO("Switch position samples disagree, returning UNKNOWN position", samples[i], samples[i - 1]);
            return CoolerSwitchPosition::UNKNOWN;
        }
    }

    return _getSwitchPosition(samples[0]);
}

uint16_t SwitchADC::adc() {
    if (samples.empty()) {
        return 0;
    }

    uint32_t sum = 0;
    for (size_t i = 0; i < samples.size(); i++) {
        sum += samples[i];
    }
    return sum / samples.size();
}
