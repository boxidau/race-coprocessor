#include "dio.h"
#include "constants.h"
#include <DebugLog.h>
#include <Bounce.h>
#include <utils.h>

DIO::DIO(
    const int* inputPins, const uint8_t inputPinCount,
    const int* outputPins, const uint8_t outputPinCount,
    const int outputCanId
) {
    for (uint8_t i = 0; i < inputPinCount; i++)
    {
        _inputPins[i] = inputPins[i];
    }
    for (uint8_t i = 0; i < outputPinCount; i++)
    {
        _outputPins[i] = outputPins[i];
    }
    txCanId = outputCanId;
    _inputPinCount = inputPinCount;
    _outputPinCount = outputPinCount;
    // rxCanId = inputCanId;
}

void DIO::setup()
{
    for (uint8_t i = 0; i < _inputPinCount; i++)
    {
        pinMode(_inputPins[i], INPUT);
    }
    for (uint8_t i = 0; i < _outputPinCount; i++)
    {
        pinMode(_outputPins[i], OUTPUT);
    }
}

void DIO::getStatusCANMessage(CAN_message_t &msg)
{
    msg.id = txCanId;
    msg.len = 1;
    inputPinBitmap = 0;
    for (int i = 0; i < _inputPinCount; i++)
    {
        inputPinBitmap |= digitalRead(_inputPins[i]) << i;
    }
    msg.buf[0] = inputPinBitmap;
    LOG_DEBUG("DIO Status: ", hexDump(msg.buf));
}

