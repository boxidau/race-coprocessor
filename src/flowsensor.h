#pragma once
#include "Arduino.h"

#define FLOW_SAMPLES 50

class FlowSensor {
private:
    uint8_t _flowSensorPin;
    float _pulsePeriodMicrosec; // pulse period representing 1Lpm
    uint32_t _timeoutMilliseconds;
    float memoizedTemp { - 200 };
    float memoizedCalibration { 0 };

    float flowRateCalibrationForTemperature(float temp);
    float ensureCalibrationSample(uint8_t idx);

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
    void updateTemp(float temp);
    uint32_t lastPulseMicros();
    uint32_t lastPulseDuration();
    uint8_t lastPulseIndex();
};