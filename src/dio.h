#include "Arduino.h"
#include <FlexCAN.h>


// Digital IO

class DIO
{
public:
    DIO(
        const int* inputPins,
        const uint8_t inputPinCount,
        const int* outputPins,
        const uint8_t outputPinCount,
        int outputCanId
    );
    void getStatusCANMessage(CAN_message_t &msg);
    // void updateFromReceivedCANMessage(CAN_message_t &msg);
    void setup();
private:
    int _inputPins[8];
    int _outputPins[8];
    uint16_t txCanId;
    byte inputPinBitmap = 0;
    uint8_t _inputPinCount = 0;
    uint8_t _outputPinCount = 0;
    // uint16_t rxCanId;
};