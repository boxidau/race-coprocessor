#include "DebugLog.h"

#include "flowsensor.h"
#include "utils.h"

static volatile uint32_t _samples[FLOW_SAMPLES];
static volatile uint8_t _idx;
static volatile bool _filled;

uint32_t getPrevSample(uint8_t offset) {
    if (!_filled && _idx < offset + 1) {
        return 0;
    }
    int8_t idx = _idx - offset - 1;
    if (idx < 0) {
        idx += FLOW_SAMPLES;
    }
    return _samples[idx];
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

float FlowSensor::flowRate() {
    __disable_irq();
    bool filled = _filled;
    uint32_t firstPulse = _samples[_idx];
    uint32_t lastPulse = getPrevSample(0);
    __enable_irq();

    if (!filled) {
        return 0;
    }
    
    uint32_t now = micros();
    uint32_t timeSincePulseSeen = MICROS_DURATION(now, lastPulse);
    if (timeSincePulseSeen > _timeoutMilliseconds * 1000) {
        // flow has stalled, reset state
        __disable_irq();
        _filled = false;
        _idx = 0;        
        __enable_irq();
        return 0;
    }

    uint32_t period = MICROS_DURATION(lastPulse, firstPulse);
    return _pulsePeriodMicrosec * (FLOW_SAMPLES - 1) / period;
}

float FlowSensor::instantaneousFlowRate() {
    __disable_irq();
    bool hasSamples = _filled || _idx > 1;
    uint32_t sample0 = getPrevSample(0);
    uint32_t sample1 = getPrevSample(1);
    __enable_irq();

    if (!hasSamples) {
        return 0;
    }

    uint32_t now = micros();
    uint32_t timeSincePulseSeen = MICROS_DURATION(now, sample0);
    if (timeSincePulseSeen > _timeoutMilliseconds * 1000) {
        return 0;
    }

    return _pulsePeriodMicrosec / MICROS_DURATION(sample0, sample1);
}

uint32_t FlowSensor::lastPulseMicros() {
    __disable_irq();
    uint32_t sample = getPrevSample(0);
    __enable_irq();
    return sample;
}

uint32_t FlowSensor::lastPulseDuration() {
    __disable_irq();
    uint32_t sample0 = getPrevSample(0);
    uint32_t sample1 = getPrevSample(1);
    __enable_irq();
    return MICROS_DURATION(sample0, sample1);
}

uint8_t FlowSensor::lastPulseIndex() {
    __disable_irq();
    uint8_t idx = _idx;
    bool filled = _filled;
    __enable_irq();

    if (idx == 0 && !filled) {
        return 0;
    }
    return idx == 0 ? FLOW_SAMPLES - 1 : idx - 1;
}