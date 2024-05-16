#pragma once
#include "Arduino.h"

#define FLOW_SAMPLES 100

class FlowSensor {
private:
    uint8_t _flowSensorPin;
    float _pulsePeriodMicrosec; // pulse period representing 1Lpm
    uint32_t _timeoutMilliseconds;
    uint32_t _measurementIntervalMilliseconds;
    float memoizedTemp { - 200 };
    float memoizedCalibration { 0 };

    float flowRateCalibrationForTemperature(float temp);
    uint32_t ensurePulsePeriodSample(uint8_t idx, uint32_t pulse, uint32_t prevPulse);

public:
    FlowSensor(
        uint8_t flowSensorPin,
        float hertzPerLpm,
        uint32_t timeoutMilliseconds,
        uint32_t measurementIntervalMilliseconds
    )
        : _flowSensorPin { flowSensorPin }
        , _pulsePeriodMicrosec { 1e6 / hertzPerLpm }
        , _timeoutMilliseconds { timeoutMilliseconds }
        , _measurementIntervalMilliseconds { measurementIntervalMilliseconds }
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