#include "Arduino.h"
#include <SD.h>
#include <Array.h>

#define NTC_ARRAY_SIZE 8000

struct NTCData {
    uint32_t time;
    uint16_t ntc1;
    uint16_t ntc1avg;
    uint16_t ntc2;
    uint16_t ntc2avg;
};

class NTCLogger
{
public:
    void ensureSetup();
    void logSamples(uint32_t time, uint16_t ntc1, uint16_t ntc1avg, uint16_t ntc2, uint16_t ntc2avg);

private:
    void flush();

    File logFile;
    bool started { false };
    bool enableLog { true } ;
    Array<NTCData, NTC_ARRAY_SIZE> ntcData;
};