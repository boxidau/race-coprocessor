#include "Arduino.h"
#include <FlexCAN.h>

#define CAN_RX true
#define CAN_TX false

#define CANLOGGER_PREALLOC_MB 1 // 1MB ~= 10 mins of logs, 18MB ~= 4 hours
#define CANLOGGER_FLUSH_MS 0

class CANLogger
{
public:
    static void logCANMessage(const CAN_message_t &message, bool rx);
    static void stringify(char *output, const CAN_message_t &message, bool rx);

private:
    static void write();
};