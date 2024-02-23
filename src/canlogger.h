#include "Arduino.h"
#include <FlexCAN.h>
#include <SD.h>
#include "stringformat.h"

#define CAN_RX true
#define CAN_TX false

#define PREALLOC_BYTES 1000000
#define FLUSH_MS 5000

class CANLogger
{
public:
    static void setup();
    static void logComment(const String line);
    static void logCANMessage(const CAN_message_t &message, bool rx);
    static void stringify(char *output, const CAN_message_t &message, bool rx);
    static bool error;

    static void logMessage(StringFormatCSV& format);
private:
    static void write(StringFormatCSV& format);
    static void write() {
        char buf[1];
        StringFormatCSV format(buf, sizeof(buf));
        write(format);
    }
};