#include "DebugLog.h"

#include "flowsensor.h"
#include "utils.h"

static volatile uint32_t _flowSamples[FLOW_SAMPLES];
static volatile uint32_t _pulsePeriodCalibratedSamples[FLOW_SAMPLES];
static volatile float _tempSamples[FLOW_SAMPLES];
static volatile float _currentTemp;
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
    return _flowSamples[idx];
}

void recordPulse() {
    uint8_t idx = _idx;
    _flowSamples[idx] = micros();
    _tempSamples[idx] = _currentTemp;
    // zero out the calibration value so we know it needs to be calculated later. want to avoid doing
    // expensive floating point math in an interrupt handler
    _pulsePeriodCalibratedSamples[idx] = 0;

    idx++;
    if (idx == FLOW_SAMPLES) {
        idx = 0;
        _filled = true;
    }
    _idx = idx;
}

void FlowSensor::setup() {
    pinMode(_flowSensorPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(_flowSensorPin), recordPulse, RISING);
}

float FlowSensor::flowRate() {
    __disable_irq();
    bool filled = _filled;
    uint8_t idx = _idx;
    __enable_irq();

    uint8_t totalPulses = filled ? FLOW_SAMPLES - 1 : idx;
    if (totalPulses < 2) {
        return 0;
    }

    uint32_t now = micros();
    uint8_t curIdx = idx > 0 ? idx - 1 : FLOW_SAMPLES - 1;
    uint32_t lastPulse = _flowSamples[curIdx];
    if (MICROS_DURATION(now, lastPulse) > _timeoutMilliseconds * 1000) {
        return 0;
    }

    uint8_t prevIdx = curIdx;
    uint8_t pulses = 0;
    uint32_t runningPeriod = 0;
    // avoid race condition: ignore last sample in case it gets updated as we're running the loop
    while (pulses < totalPulses - 1) {
        prevIdx = curIdx > 0 ? curIdx - 1 : FLOW_SAMPLES - 1;
        uint32_t pulse = _flowSamples[prevIdx];
        if (MICROS_DURATION(now, pulse) > _measurementIntervalMilliseconds * 1000) {
            break;
        }

        runningPeriod += ensurePulsePeriodSample(curIdx, lastPulse, pulse);
        curIdx = prevIdx;
        lastPulse = pulse;
        pulses++;
    } 

    return pulses > 0 ? _pulsePeriodMicrosec * pulses / runningPeriod : 0;
}

float FlowSensor::instantaneousFlowRate() {
    __disable_irq();
    bool filled = _filled;
    uint8_t idx = _idx;
    __enable_irq();

    if (!filled && idx <= 1) {
        return 0;
    }

    uint8_t sample0idx = idx > 0 ? idx - 1 : FLOW_SAMPLES - 1;
    uint8_t sample1idx = sample0idx > 0 ? sample0idx - 1 : FLOW_SAMPLES - 1;
    uint32_t sample0 = _flowSamples[sample0idx];
    uint32_t sample1 = _flowSamples[sample1idx];

    uint32_t now = micros();
    uint32_t timeSincePulseSeen = MICROS_DURATION(now, sample1);
    if (timeSincePulseSeen > _timeoutMilliseconds * 1000) {
        return 0;
    }

    uint32_t pulsePeriod = ensurePulsePeriodSample(sample0idx, sample0, sample1);
    return _pulsePeriodMicrosec / pulsePeriod;
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

void FlowSensor::updateTemp(float temp) {
    __disable_irq();
    _currentTemp = temp;
    __enable_irq();
}

float FlowSensor::flowRateCalibrationForTemperature(float temp) {
    if (memoizedTemp == temp) {
        // memoize the result, since we're likely to call this function for multiple pulses with the same temperature
        return memoizedCalibration;
    }

    if (temp < -2 || temp > 50) {
        LOG_WARN("Temperature out of flow meter calibration range, results will be inaccurate");
    }

    // 4th order polynomial fit
    memoizedCalibration = (((-0.00000012526 * temp + 0.000018010) * temp - 0.00089138) * temp + 0.016267) * temp + 0.90948;
    memoizedTemp = temp;
    return memoizedCalibration;
}

// calculate and store the calibrated pulse period between `pulse` and `prevPulse`. `idx` should be the index
// of `pulse`, where the result will be stored.
uint32_t FlowSensor::ensurePulsePeriodSample(uint8_t idx, uint32_t pulse, uint32_t prevPulse) {
    uint32_t pulsePeriod = _pulsePeriodCalibratedSamples[idx];
    if (pulsePeriod == 0) {
        // calibrate each pulse using the temperature at which it was recorded
        float calibration = flowRateCalibrationForTemperature(_tempSamples[idx]);
        pulsePeriod = MICROS_DURATION(pulse, prevPulse) / calibration;
    }
    _pulsePeriodCalibratedSamples[idx] = pulsePeriod;
    return pulsePeriod;
}
