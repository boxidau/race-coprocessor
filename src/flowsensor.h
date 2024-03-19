#pragma once
#include "Arduino.h"

#define FLOW_SAMPLES 16
#define FLOW_DEBOUNCE_MS 0 // disable debounce

class FlowSensor {
private:
    uint32_t _flowSensorPin, _pulsePeriodMicros;
    uint _timeoutMilliseconds;

public:
    FlowSensor(
        uint flowSensorPin,
        float pulsesPerLiter,
        uint timeoutMilliseconds
    )
        : _flowSensorPin { flowSensorPin }
        , _pulsePeriodMicros { uint32_t(1e9 / pulsesPerLiter) }
        , _timeoutMilliseconds { timeoutMilliseconds }
    {
    };

    void setup();
    uint16_t flowRate();
    uint32_t lastPulseMicros();
    uint32_t lastPulseDuration();
    uint8_t lastPulseIndex();
};