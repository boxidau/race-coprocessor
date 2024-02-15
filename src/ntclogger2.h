#include "Arduino.h"
#include <SdFat.h>
#include <Array.h>

#define NTC_ARRAY_SIZE 20000
#define LINEBUF_SIZE 8000

class NTCLogger2
{
public:
    void setup();
    void logSamples(uint16_t ntc1, uint16_t ntc2, uint16_t ntcDifferential, uint16_t ntc3, uint16_t ntc4, uint16_t ambient);
    uint64_t msSinceStarted() { return time64 / 1000; };

private:
    void flush();

    File32 logFile;
    unsigned long prevTime { 0 };
    int bufWritten { 0 };
    uint64_t time64 { 0 };
    bool started { false };
    bool enableLog { true } ;
    //Array<NTCData, NTC_ARRAY_SIZE> ntcData;
    char* lineBuf;
};
