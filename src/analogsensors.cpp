#include "analogsensors.h"
#include "constants.h"
#include <DebugLog.h>


AnalogSensors::AnalogSensors(const int *pins, const uint8_t pinCount, int outputCanId)
{
    for (uint8_t i = 0; i < pinCount; i++)
    {
        analogPins[i] = pins[i];
    }
    analogPinsCount = pinCount;
    canId = outputCanId;
}

void AnalogSensors::setup()
{
    for (uint8_t i = 0; i < analogPinsCount; i++)
    {
        LOG_DEBUG("Setting pin mode INPUT for pin", analogPins[i]);
        pinMode(analogPins[i], INPUT);
    }
}

void AnalogSensors::getCANMessage(CAN_message_t &msg)
{
    msg.id = canId;
    msg.len = analogPinsCount;
    msg.flags = {};

    int adc_val = 0;

    for (uint8_t i = 0; i < analogPinsCount; i++)
    {
        adc_val = analogRead(analogPins[i]);
        LOG_DEBUG("ADC ", analogPins[i], "=", adc_val);

        msg.buf[i*2] = adc_val >> 8;
        msg.buf[i*2 + 1] = adc_val & 0xFF;
    }
}