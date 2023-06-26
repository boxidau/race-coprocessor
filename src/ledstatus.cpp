#include "constants.h"
#include "ledstatus.h"
#include "Arduino.h"

#include <Metro.h>
#include <DebugLog.h>

byte LEDStatus::flashCode = NO_ERROR;
uint8_t LEDStatus::flashCodeIndex = 0;

static Metro ledTimer = Metro(300);

void LEDStatus::setup()
{
    pinMode(LED1, OUTPUT);
}

void LEDStatus::loop()
{
    if (ledTimer.check())
    {
        int bit = flashCodeIndex % 21 / 2;
        if (flashCodeIndex % 2 == 1)
        {
            // LED off between bit indication
            // odd sequence numbers
            digitalWrite(LED1, LOW);
        }
        else if (bit > 7)
        {
            // break to indicate the end/start of flash sequence
            // bit is out of range
            digitalWrite(LED1, LOW);
        }
        else
        {
            // display the error code bit
            digitalWriteFast(LED1, bitRead(flashCode, bit));
        }
        flashCodeIndex++;
    }
}

void LEDStatus::setError(byte error)
{
    flashCode |= error;
}

void LEDStatus::clearError(byte error)
{
    flashCode &= ~error;
}

void LEDStatus::setError(byte error, bool setOrClear)
{
    if (setOrClear)
    {
        setError(error);
    }
    else
    {
        clearError(error);
    }
}

byte LEDStatus::getError()
{
    return flashCode;
}
