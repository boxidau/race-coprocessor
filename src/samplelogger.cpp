#include "samplelogger.h"

#include <DebugLog.h>
#include <Metro.h>
#include <TimeLib.h>
#include <SD.h>

#include "sdlogger.h"
#include "stringformat.h"

void SampleLogger::ensureSetup(const char* header)
{
    if (enableLog) {
        return;
    }

    if (!SDLogger::createLogfile('s', logFile)) {
        return;
    }

    logFile.println(header);
    enableLog = true;
}

void SampleLogger::logSamples(uint32_t time, uint16_t sample1, uint16_t sample2, uint16_t sample3, uint16_t sample4) {
    if (!enableLog) {
        return;
    }

    SampleData data;
    data.time = time;
    data.sample1 = sample1;
    data.sample2 = sample2;
    data.sample3 = sample3;
    //data.sample4 = sample4;
    sampleData.push_back(data);

    if (sampleData.full()) {
        flush();
    }
}

void SampleLogger::flush()
{
    if (!enableLog) {
        return;
    }

    for (size_t i = 0; i < sampleData.size(); i++) {
        const SampleData& data = sampleData[i];

        char message[128];
        StringFormatCSV format(message, sizeof(message));
        format.formatUnsignedInt(data.time);
        format.formatUnsignedInt(data.sample1);
        format.formatUnsignedInt(data.sample2);
        format.formatUnsignedInt(data.sample3);
        //format.formatUnsignedInt(data.sample4);

        logFile.write(format.finish(), format.length());
    }

    sampleData.clear();
    logFile.flush();
}
