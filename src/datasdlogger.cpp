#include "datasdlogger.h"

#include <DebugLog.h>
#include <Metro.h>
#include <TimeLib.h>
#include <SD.h>

#include "sdlogger.h"
#include "clocktime.h"

static char lineBuffer[512];
static FsFile logFile;
static bool enableLog;
static Metro retrySDTimer = Metro(FLUSH_MS);

void DataSDLogger::setup()
{
    if (!SDLogger::createLogfile(0, logFile)) {
        return;
    }

#if PREALLOC_MB
    bool didPrealloc = SDLogger::preAlloc(logFile, PREALLOC_MB * 1000000);
    if (!didPrealloc) {
        return;
    }
#endif

    enableLog = true;
}

void DataSDLogger::logComment(const String line)
{
    char strBuf[241];
    line.toCharArray(strBuf, 240);

    snprintf(lineBuffer, 254, "%0.3f CXX %s", ClockTime::secSinceEpoch(), strBuf);
    //write();
}

bool DataSDLogger::logData(const char* data, size_t len)
{
    bool tick = retrySDTimer.check();
    if (!enableLog)
    {
        if (tick)
        {
            DataSDLogger::setup();
        }
        return false;
    }

    uint32_t m = logFile.write(data, len);
    if (m != len)
    {
        LOG_INFO("Failed write, bytes written", m, "desired length", len);
        enableLog = false;
        return false;
    }

    #if FLUSH_MS
        if (tick)
        {
            logFile.flush();
            return true;
        }
    #endif

    return false;
}

bool DataSDLogger::logData(const char* data)
{
    return logData(data, strlen(data));
}
