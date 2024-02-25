#include "flowsensor.h"

static volatile uint32_t _samples[FLOW_SAMPLES];
static volatile uint8_t _idx;
static volatile bool _filled;

#define MICROS_DURATION(a, b) ((a) > (b) ? (a) - (b) : (a) + (UINT32_MAX - (b)) + 1)

uint32_t getPrevSample() {
    uint8_t idx = _idx;
    if (idx == 0 && !_filled) {
        return 0;
    }
    return _samples[idx == 0 ? FLOW_SAMPLES - 1 : idx - 1];
}

void recordPulse() {
    uint32_t sample = micros();
    // debounce for 10ms
    uint32_t prevSample = getPrevSample();
    uint32_t duration = MICROS_DURATION(sample, prevSample);
    if (duration < FLOW_DEBOUNCE_MS * 1000) {
        return;
    }

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
    uint32_t lastPulse = getPrevSample();
    
    uint32_t now = micros();
    // handle overflow
    uint32_t timeSincePulseSeen = MICROS_DURATION(now, lastPulse);
    if (timeSincePulseSeen > _timeoutMilliseconds * 1000) {
        return 0;
    }

    uint32_t period = MICROS_DURATION(lastPulse, firstPulse);
    return _pulsePeriodMicros / (period / (FLOW_SAMPLES - 1));
}

unsigned long FlowSensor::lastPulseMicros() {
    return getPrevSample();
}
