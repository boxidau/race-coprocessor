#include "Arduino.h"
#include <FlexCAN.h>

class AnalogSensors
{
public:
    AnalogSensors(const int *pins, const uint8_t pinCount, int outputCanId);
    void getCANMessage(CAN_message_t &msg);
    void setup();
private:
    int analogPins[4];
    uint16_t canId;
    uint8_t analogPinsCount;
};