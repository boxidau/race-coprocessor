#include "Arduino.h"
#include <SD.h>
#include <Array.h>

#define NTC_ARRAY_SIZE 15000

struct NTCData {
    uint32_t time;
    uint16_t sample1;
    uint16_t sample2;
    uint16_t sample3;
    //uint16_t sample4;
};

class NTCLogger
{
public:
    void ensureSetup(const char* header);
    void logSamples(uint32_t time, uint16_t sample1, uint16_t sample2, uint16_t sample3, uint16_t sample4);

private:
    void flush();

    File logFile;
    bool started { false };
    bool enableLog { false } ;
    Array<NTCData, NTC_ARRAY_SIZE> ntcData;
};