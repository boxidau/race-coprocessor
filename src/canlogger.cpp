#include "canlogger.h"

#include "clocktime.h"

static char lineBuffer[512];

void CANLogger::stringify(char output[255], const CAN_message_t &message, bool rx) {
    sprintf(output, "%0.3f %d%s%s %lu ",
            ClockTime::secSinceEpoch(),
            1, // hardcode canbus 1 for now
            rx ? "R" : "T",
            message.ext ? "29" : "11",
            message.id
    );

    for (int i = 0; i < message.len; i++) {
        char hexBuf[3];
        sprintf(hexBuf, "%02X",  message.buf[i]);
        strcat(output, hexBuf);
        if (i != message.len-1) {
            strcat(output, " ");
        }
    }
}

void CANLogger::logCANMessage(const CAN_message_t &message, bool rx)
{
    stringify(lineBuffer, message, rx);
    write();
}

void CANLogger::write() {
    // placeholder
}
