#include "Arduino.h"
#include <FlexCAN.h>
#include <SD.h>
#include "stringformat.h"

#define CAN_RX true
#define CAN_TX false

#define PREALLOC_MB 1 // 18MB ~= 4 hours of logs
#define FLUSH_MS 10000

class CANLogger
{
public:
    static void setup();
    static void logComment(const String line);
    static void logCANMessage(const CAN_message_t &message, bool rx);
    static void stringify(char *output, const CAN_message_t &message, bool rx);
    static bool error;

    static bool logMessage(StringFormatCSV& format);
private:
    static bool write(StringFormatCSV& format);
    static void write() {
        char buf[1];
        StringFormatCSV format(buf, sizeof(buf));
        write(format);
    }
};