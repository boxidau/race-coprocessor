#include "Arduino.h"
#include <SD.h>
#include <Array.h>

#if NTC_DEBUG
#define SAMPLE_ARRAY_SIZE 10000
#else
#define SAMPLE_ARRAY_SIZE 500 // every 20s
#endif

struct SampleData {
    uint32_t time;
    uint16_t sample1;
    uint32_t sample2;
    int32_t sample3;
    int32_t sample4;
    uint16_t sample5;
    uint16_t sample6;
    uint16_t sample7;
    uint16_t sample8;
};

class SampleLogger
{
public:
    void ensureSetup(const char* header);
    void logSamples(uint32_t time, uint16_t sample1, uint32_t sample2, int32_t sample3, int32_t sample4, uint16_t sample5, uint16_t sample6, uint16_t sample7, uint16_t sample8);

private:
    void flush();

    FsFile logFile;
    bool enableLog { false } ;
    Array<SampleData, SAMPLE_ARRAY_SIZE> sampleData;
};