#include "pwm.h"

#define PWM_MAX 65535

PWMOutput::PWMOutput(
    const uint pin, 
    const bool inverted,
    const float frequency
)
    : _pin { pin }
    , _inverted { inverted }
    , _frequency { frequency } 
{
};

void PWMOutput::setup() {
    analogWriteFrequency(_pin, _frequency);
    pinMode(_pin, OUTPUT);
    set(0);
};

const uint16_t PWMOutput::value() {
    return _value;
};

const uint8_t PWMOutput::percent() {
    return (_value / PWM_MAX) * 100;
};

void PWMOutput::set(const uint16_t v) {
    _value = v;
    if (_inverted) {
        analogWrite(_pin, PWM_MAX-_value);
    } else {
        analogWrite(_pin, _value);
    }
};

void PWMOutput::setPercent(const uint8_t p) {
    set((uint16_t)((constrain(p, 0, 100) / 100.0) * PWM_MAX));
};

void PWMOutput::setBoolean(const bool b) {
    if (b) {
        set(PWM_MAX);
    } else {
        set(0);
    }
}
