#include "Arduino.h"

#include <SPI.h>
#include "constants.h"

#include <FlexCAN.h>

struct ThermocoupleMessage
{
    uint16_t celsius;
    byte error;
};

class Thermocouple
{
public:
    static void setup();
    static void getCANMessage(CAN_message_t &msg);
    static void readData(const int chipSelectPin, ThermocoupleMessage &msg);
    static byte getError();
};

