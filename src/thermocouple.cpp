#include "thermocouple.h"
#include "utils.h"

#include <DebugLog.h>

static SPISettings SPISettingsEGT(4000000, MSBFIRST, SPI_MODE0);
static byte rxBuffer[4];

static ThermocoupleMessage tc1;
static ThermocoupleMessage tc2;

void Thermocouple::setup()
{
    pinMode(SPI_CS_TC1, OUTPUT);
    digitalWrite(SPI_CS_TC1, HIGH);
    pinMode(SPI_CS_TC2, OUTPUT);
    digitalWrite(SPI_CS_TC2, HIGH);
}

void Thermocouple::readData(const int chipSelectPin, ThermocoupleMessage &msg)
{
    SPI.beginTransaction(SPISettingsEGT);
    digitalWriteFast(chipSelectPin, LOW);
    SPI.transfer(rxBuffer, 4);
    digitalWriteFast(chipSelectPin, HIGH);
    SPI.endTransaction();

    if (MOCK_DATA)
    {
        rxBuffer[0] = 0x3E;
        rxBuffer[1] = 0x80;
        rxBuffer[2] = 0x19;
    }

    LOG_TRACE("Thermocouple RX", chipSelectPin, hexDump(rxBuffer));

    msg.celsius = ((rxBuffer[0] & 0x7F) << 4 | rxBuffer[1] >> 4);
    if (rxBuffer[0] & 0x80)
    {
        LOG_WARN("Truncated negative temp");
        msg.celsius = 0;
    }

    LOG_TRACE("Thermocouple Value: ", msg.celsius);

    msg.error = rxBuffer[3] & 0x7;
    if (rxBuffer[3] & 0x4)
    {
        LOG_DEBUG("Thermocouple Error: Short to VCC on", chipSelectPin);
        msg.celsius = 4;
    }

    if (rxBuffer[3] & 0x2) 
    {
        LOG_DEBUG("Thermocouple Error: Short to GND on", chipSelectPin);
        msg.celsius = 2;
    }

    if (rxBuffer[3] & 0x1)
    {
        LOG_DEBUG("Thermocouple Error: Open circuit on", chipSelectPin);
        msg.celsius = 1;
    }
}

byte Thermocouple::getError()
{
    return tc1.error | tc2.error;
}

void Thermocouple::getCANMessage(CAN_message_t &msg)
{
    Thermocouple::readData(SPI_CS_TC1, tc1);
    Thermocouple::readData(SPI_CS_TC2, tc2);
    msg.id = CANID_THERMOCOUPLE;
    msg.len = 6;
    msg.flags = {};
    msg.buf[0] = tc1.celsius >> 8;
    msg.buf[1] = tc1.celsius & 0xFF;
    msg.buf[2] = tc1.error;
    msg.buf[3] = tc2.celsius >> 8;
    msg.buf[4] = tc2.celsius & 0xFF;
    msg.buf[5] = tc2.error;
}