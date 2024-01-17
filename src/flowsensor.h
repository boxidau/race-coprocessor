#pragma once
#include "Arduino.h"
#include <Bounce.h>

#define FLOW_SAMPLES 16

class FlowSensor {
private:
    Bounce _flowRateInput;
    uint32_t _flowSensorPin, _pulsePeriodMicros, _pulseInterval;
    static volatile uint32_t _samples[FLOW_SAMPLES];
    static volatile uint8_t _idx { 0 };
    static volatile bool _filled { false };
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
    void loop();
    uint16_t flowRate();
};