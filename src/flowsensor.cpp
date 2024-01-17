#include "flowsensor.h"

void recordPulse() {
    FlowSensor::_samples[FlowSensor::_idx++] = micros();
    if (FlowSensor::_idx == FLOW_SAMPLES) {
        FlowSensor::_idx == 0;
        FlowSensor::_filled = true;
    }
}

void setup() {
    pinMode(_flowSensorPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(_flowSensorPin), recordPulse, RISING);
}

void FlowSensor::loop() {
    return;

    if (_flowRateInput.update() && _flowRateInput.risingEdge()) {
        _pulseInterval = micros() - _lastPulse;
        _samples[_idx % FLOW_SAMPLES] = _pulseInterval;
        _idx++;
        _lastPulse += _pulseInterval;
        uint sum = 0;
        for (uint8_t i = 0; i < FLOW_SAMPLES; i++) {
            sum += _samples[i];
        }
        uint16_t avg = (sum / FLOW_SAMPLES);
        _flowRate = _pulsePeriodMicros / avg;
    }

    // if we haven't seen a pulse in a long time
    // reset flow rate to 0
    if (((micros() - _lastPulse) / 1000) > _timeoutMilliseconds) {
        _flowRate = 0;
        // LOG_ERROR("Flow sensor timeout");
    }
}

uint16_t FlowSensor::flowRate() {
    // old way
    //return _flowRate;

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
