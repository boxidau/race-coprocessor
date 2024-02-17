#pragma once
#include "Arduino.h"
#include <Bounce.h>

#define FLOW_SAMPLES 16

class FlowSensor {
private:
    Bounce _flowRateInput;
    uint32_t _flowSensorPin, _pulsePeriodMicros, _pulseInterval;
    uint16_t _flowRate { 0 }; // mL/min
    uint _timeoutMilliseconds;

public:
    FlowSensor(
        uint flowSensorPin,
        double pulsesPerLiter,
        uint timeoutMilliseconds = 1000,
        uint debounceMs = 1
    )
        : _flowRateInput { Bounce(flowSensorPin, debounceMs) }
        , _flowSensorPin { flowSensorPin }
        , _pulsePeriodMicros { (uint32_t)(1000000000 / pulsesPerLiter) }
        , _timeoutMilliseconds { timeoutMilliseconds }
    {
    };

    void setup();
    uint16_t flowRate();
    unsigned long lastPulseMicros();
};