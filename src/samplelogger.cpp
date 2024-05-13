#include "samplelogger.h"

#include <DebugLog.h>
#include <Metro.h>
#include <TimeLib.h>
#include <SD.h>

#include "sdlogger.h"
#include "stringformat.h"
#include "clocktime.h"

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

void SampleLogger::logSamples(uint32_t time, uint16_t sample1, uint32_t sample2, int32_t sample3, int32_t sample4, uint16_t sample5, uint16_t sample6, uint16_t sample7, uint16_t sample8) {
    if (!enableLog) {
        return;
    }

    SampleData data;
    data.time = time;
    data.sample1 = sample1;
    data.sample2 = sample2;
    data.sample3 = sample3;
    data.sample4 = sample4;
    data.sample5 = sample5;
    data.sample6 = sample6;
    data.sample7 = sample7;
    data.sample8 = sample8;
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

        StringFormat<128, ','> format;
        format.formatUnsignedInt(data.time);
        format.formatUnsignedInt(data.sample1);
        format.formatUnsignedInt(data.sample2);
        format.formatInt(data.sample3);
        format.formatInt(data.sample4);
        format.formatUnsignedInt(data.sample5);
        format.formatUnsignedInt(data.sample6);
        format.formatUnsignedInt(data.sample7);
        format.formatUnsignedInt(data.sample8);

        logFile.write(format.finish(), format.length());
    }

    LOG_INFO("Flushed sample log at", ClockTime::secSinceEpoch());
    sampleData.clear();
    logFile.flush();
}
