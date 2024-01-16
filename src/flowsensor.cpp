#include "flowsensor.h"

void FlowSensor::loop() {
    if (_flowRateInput.update() && _flowRateInput.fallingEdge()) {
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