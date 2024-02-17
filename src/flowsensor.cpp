#include "flowsensor.h"

static volatile uint32_t _samples[FLOW_SAMPLES];
static volatile uint8_t _idx;
static volatile bool _filled;

void recordPulse() {
    _samples[_idx++] = micros();
    if (_idx == FLOW_SAMPLES) {
        _idx = 0;
        _filled = true;
    }
}

void FlowSensor::setup() {
    pinMode(_flowSensorPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(_flowSensorPin), recordPulse, RISING);
}

uint16_t FlowSensor::flowRate() {
    if (!_filled) {
        return 0;
    }

    uint32_t firstPulse = _samples[_idx];
    uint32_t lastPulse = _samples[_idx == 0 ? FLOW_SAMPLES - 1 : _idx - 1];
    
    uint32_t now = micros();
    // handle overflow
    uint32_t timeSincePulseSeen = now > lastPulse ? now - lastPulse : now + (UINT32_MAX - lastPulse);
    if (timeSincePulseSeen / 1000 > _timeoutMilliseconds) {
        return 0;
    }

    uint32_t period = lastPulse > firstPulse ? lastPulse - firstPulse : lastPulse + (UINT32_MAX - firstPulse);
    return _pulsePeriodMicros / (period / (FLOW_SAMPLES - 1));
}

unsigned long FlowSensor::lastPulseMicros() {
    return _samples[_idx == 0 ? FLOW_SAMPLES - 1 : _idx - 1];
}
