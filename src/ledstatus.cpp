#include "constants.h"
#include "ledstatus.h"
#include "Arduino.h"

#include <Metro.h>
#include <DebugLog.h>

void LEDStatus::setup()
{
    pinMode(rhythmLED, OUTPUT);
    pinMode(statusLED, OUTPUT);
    analogWriteFrequency(rhythmLED, 20000);
    analogWriteFrequency(statusLED, 20000);
}

void LEDStatus::loop()
{
    static Metro ledTimer = Metro(333);
    static uint8_t flashCodeIndex = 0;

    if (ledTimer.check())
    {
        // pulse the rhythm (clock) LED to make it easy to read the status LED
        analogWrite(rhythmLED, LED_PWM_MAX);
        if (flashCodeIndex % 2 == 1)
        {
            // LED off between bit indication
            // odd sequence numbers
            analogWrite(statusLED, 0);
            analogWrite(rhythmLED, 0);
        }
        else if (flashCodeIndex >= 16)
        {
            // break to indicate the end/start of flash sequence
            // bit is out of range
            analogWrite(statusLED, 0);
            analogWrite(rhythmLED, 0);
        }
        else
        {
            // display the error code bit
            analogWrite(statusLED, bitRead(error, flashCodeIndex / 2) ? LED_PWM_MAX : 0);
        }
        flashCodeIndex++;
        if (flashCodeIndex >= 18) flashCodeIndex = 0;
    }
}
