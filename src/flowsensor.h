#pragma once
#include "Arduino.h"

#define FLOW_SAMPLES 16
#define FLOW_DEBOUNCE_MS 0

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
    unsigned long lastPulseMicros();
    unsigned long lastPulseDuration();
    uint8_t lastPulseIndex();
};