#include "flowsensor.h"
#include "utils.h"

#include "DebugLog.h"
static volatile uint32_t _samples[FLOW_SAMPLES];
static volatile uint8_t _idx;
static volatile bool _filled;

uint32_t getPrevSample(uint8_t offset) {
    uint8_t idx = _idx;
    if (idx == 0 && !_filled) {
        return 0;
    }
    return _samples[uint8_t(idx - offset - 1) % FLOW_SAMPLES];
}

void recordPulse() {
    uint32_t sample = micros();

    // debounce
    uint32_t prevSample = getPrevSample(0);
    uint32_t duration = MICROS_DURATION(sample, prevSample);
    if (duration < FLOW_DEBOUNCE_MS * 1000) {
        return;
    }

    _samples[_idx++] = sample;
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
    uint32_t lastPulse = getPrevSample(0);
    
    uint32_t now = micros();
    uint32_t timeSincePulseSeen = MICROS_DURATION(now, lastPulse);
    if (timeSincePulseSeen > _timeoutMilliseconds * 1000) {
        // flow has stalled, reset state
        _filled = false;
        _idx = 0;        
        return 0;
    }

    uint32_t period = MICROS_DURATION(lastPulse, firstPulse);
    return min((uint64_t) _pulsePeriodMicros * (FLOW_SAMPLES - 1) / period, UINT16_MAX);
}

unsigned long FlowSensor::lastPulseMicros() {
    return getPrevSample(0);
}

unsigned long FlowSensor::lastPulseDuration() {
    return MICROS_DURATION(getPrevSample(0), getPrevSample(1));
}

uint8_t FlowSensor::lastPulseIndex() {
    uint8_t idx = _idx;
    if (idx == 0 && !_filled) {
        return 0;
    }
    return idx == 0 ? FLOW_SAMPLES - 1 : idx - 1;
}