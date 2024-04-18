#pragma once
#include "Arduino.h"

#define FLOW_SAMPLES 16
#define FLOW_DEBOUNCE_MS 0 // disable debounce

class FlowSensor {
private:
    uint8_t _flowSensorPin;
    float _pulsePeriodMicrosec; // pulse period representing 1Lpm
    uint32_t _timeoutMilliseconds;

public:
    FlowSensor(
        uint8_t flowSensorPin,
        float hertzPerLpm,
        uint32_t timeoutMilliseconds
    )
        : _flowSensorPin { flowSensorPin }
        , _pulsePeriodMicrosec { 1e6 / hertzPerLpm }
        , _timeoutMilliseconds { timeoutMilliseconds }
    {
    };

    void setup();
    float flowRate();
    float instantaneousFlowRate();
    uint32_t lastPulseMicros();
    uint32_t lastPulseDuration();
    uint8_t lastPulseIndex();
};