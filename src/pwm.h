#include "Arduino.h"

class PWMOutput {
private:
    uint _pin;
    uint16_t _value { 0 };
    bool _inverted { false };
    float _frequency { 24414 };
public:
    PWMOutput(
        const uint pin,
        const bool inverted = false,
        const float frequency = 24414
    );
    void setup();
    const uint16_t value();
    const uint8_t percent();
    void set(const uint16_t v);
    void setPercent(const uint8_t percent);
    void setBoolean(const bool b);
};