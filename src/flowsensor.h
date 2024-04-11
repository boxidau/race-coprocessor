#pragma once
#include "Arduino.h"

#define FLOW_SAMPLES 16
#define FLOW_DEBOUNCE_MS 0 // disable debounce

class FlowSensor {
private:
    uint8_t _flowSensorPin;
    float _bufferPeriodMicros;
    uint32_t _timeoutMilliseconds;

public:
    FlowSensor(
        uint8_t flowSensorPin,
        float pulsesPerLiter,
        uint32_t timeoutMilliseconds
    )
        : _flowSensorPin { flowSensorPin }
        , _bufferPeriodMicros { 1e9 / pulsesPerLiter * (FLOW_SAMPLES - 1) }
        , _timeoutMilliseconds { timeoutMilliseconds }
    {
    };

    void setup();
    uint16_t flowRate();
    uint32_t lastPulseMicros();
    uint32_t lastPulseDuration();
    uint8_t lastPulseIndex();
};