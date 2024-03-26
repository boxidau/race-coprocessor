#include "Arduino.h"
#include <SD.h>
#include <Array.h>

#define SAMPLE_ARRAY_SIZE 15000

struct SampleData {
    uint32_t time;
    uint16_t sample1;
    uint16_t sample2;
    uint16_t sample3;
    //uint16_t sample4;
};

class SampleLogger
{
public:
    void ensureSetup(const char* header);
    void logSamples(uint32_t time, uint16_t sample1, uint16_t sample2, uint16_t sample3, uint16_t sample4);

private:
    void flush();

    FsFile logFile;
    bool enableLog { false } ;
    Array<SampleData, SAMPLE_ARRAY_SIZE> sampleData;
};