#define LED_PWM_MAX     1600

#include "Arduino.h"

class LEDStatus
{
private:
    uint8_t rhythmLED, statusLED;
public:
    LEDStatus(uint8_t _rhythmLED, uint8_t _statusLED)
      : rhythmLED { _rhythmLED }, statusLED { _statusLED }
    {};
    void setup();
    void loop();
    byte error;
};