#include "thermocouple.h"
#include "utils.h"

#include <DebugLog.h>

const SPISettings MAX31855KASASettings(4000000, MSBFIRST, SPI_MODE0);

void Thermocouple::setup()
{
    SPI.begin();
    pinMode(SPI_CS_TC1, OUTPUT);
    digitalWrite(SPI_CS_TC1, HIGH);
    pinMode(SPI_CS_TC2, OUTPUT);
    digitalWrite(SPI_CS_TC2, HIGH);
}

void Thermocouple::readData(const int chipSelectPin, ThermocoupleMessage &msg)
{
    static byte rxBuffer[4];
    SPI.beginTransaction(MAX31855KASASettings);
    digitalWriteFast(chipSelectPin, LOW);
    SPI.transfer(rxBuffer, 4);
    digitalWriteFast(chipSelectPin, HIGH);
    SPI.endTransaction();

    if (MOCK_DATA)
    {
        rxBuffer[0] = 0x0A;
        rxBuffer[1] = 0x88;
        rxBuffer[2] = 0x19;
        rxBuffer[3] = 0x80;
    }

    LOG_TRACE("Thermocouple RX", chipSelectPin, hexDump(rxBuffer));

    msg.scaledTemperature = ((rxBuffer[0] & 0x7F) << 6 | rxBuffer[1] >> 2);
    if (rxBuffer[0] & 0x80) msg.scaledTemperature *= -1;
    msg.temperature = scalbn(msg.scaledTemperature, -4);
    LOG_TRACE("Thermocouple Value [%d]: %.02f", chipSelectPin, msg.temperature);
    
    msg.scaledInternalTemperature = ((rxBuffer[2] & 0x7F) << 4 | rxBuffer[3] >> 4);
    if (rxBuffer[2] & 0x80) msg.scaledInternalTemperature *= -1;
    msg.internalTemperature = scalbn(msg.scaledInternalTemperature, -2);
    LOG_TRACE("Internal Temp [%d]: %.02f", chipSelectPin, msg.internalTemperature);

    if (rxBuffer[1] & 0x1) LOG_WARN("Thermocouple fault on ", chipSelectPin);

    msg.error = rxBuffer[3] & 0x7;
    if (rxBuffer[3] & 0x4)
    {
        LOG_WARN("Thermocouple Error: Short to VCC on", chipSelectPin);
    }

    if (rxBuffer[3] & 0x2) 
    {
        LOG_WARN("Thermocouple Error: Short to GND on", chipSelectPin);
    }

    if (rxBuffer[3] & 0x1)
    {
        LOG_WARN("Thermocouple Error: Open circuit on", chipSelectPin);
    }
}

// void Thermocouple::getCANMessage(CAN_message_t &msg)
// {
//     Thermocouple::readData(SPI_CS_TC1, tc1);
//     Thermocouple::readData(SPI_CS_TC2, tc2);
//     msg.id = CANID_THERMOCOUPLE;
//     msg.len = 6;
//     msg.flags = {};
//     msg.buf[0] = tc1.celsius >> 8;
//     msg.buf[1] = tc1.celsius & 0xFF;
//     msg.buf[2] = tc1.error;
//     msg.buf[3] = tc2.celsius >> 8;
//     msg.buf[4] = tc2.celsius & 0xFF;
//     msg.buf[5] = tc2.error;
// }