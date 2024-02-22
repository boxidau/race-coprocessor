#include "Arduino.h"
#include <SdFat.h>
#include <Array.h>

#define NTC_ARRAY_SIZE 1000

struct NTCData {
    uint32_t time;
    uint16_t ntc1;
    uint16_t ntc1avg;
    uint16_t ambient;
    uint16_t ambientavg;
};

class NTCLogger2
{
public:
    void setup();
    void logSamples(uint16_t ntc1, uint16_t ntc1avg, uint16_t ambient, uint16_t ambientavg);
    uint64_t msSinceStarted() { return time64 / 1000; };

private:
    void flush();

    FsFile logFile;
    unsigned long prevTime { 0 };
    int bufWritten { 0 };
    uint64_t time64 { 0 };
    bool started { false };
    bool enableLog { true } ;
    Array<NTCData, NTC_ARRAY_SIZE> ntcData;
    char* lineBuf;
};
