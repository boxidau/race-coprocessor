#include "Arduino.h"
#include <SD.h>
#include <Array.h>

#define NTC_ARRAY_SIZE 10000

struct NTCData {
    uint32_t time;
    uint16_t ntc1a;
    uint16_t ntc1b;
    uint16_t ntc2;
    uint16_t ntcDifferential;
    uint16_t ntc3;
    uint16_t ntc4;
    uint16_t ambient;
};

class NTCLogger
{
public:
    void setup();
    void logSamples(uint16_t ntc1a, uint16_t ntc1b, uint16_t ntc2, uint16_t ntcDifferential, uint16_t ntc3, uint16_t ntc4, uint16_t ambient);
    uint64_t msSinceStarted() { return time64 / 1000; };

private:
    void flush();

    File logFile;
    unsigned long prevTime { 0 };
    int bufWritten { 0 };
    uint64_t time64 { 0 };
    bool started { false };
    bool enableLog { true } ;
    Array<NTCData, NTC_ARRAY_SIZE> ntcData;
    char* lineBuf;
};