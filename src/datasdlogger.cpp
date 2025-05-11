#include "datasdlogger.h"

#include <DebugLog.h>
#include <Metro.h>
#include <TimeLib.h>
#include <SD.h>

#include "sdlogger.h"
#include "clocktime.h"

#if PREALLOC_MB
#define PREALLOC_BYTES (PREALLOC_MB * 1000000)
#endif

static char lineBuffer[512];
static FsFile logFile;
static bool enableLog;
static Metro flushTimer = Metro(FLUSH_MS);
static uint32_t bytesWritten;

void DataSDLogger::setup()
{
    if (!SDLogger::createLogfile(0, logFile)) {
        return;
    }

#if PREALLOC_MB
    bool didPrealloc = SDLogger::preAlloc(logFile, PREALLOC_BYTES);
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
    if (!enableLog) {
        DataSDLogger::setup();
        if (!enableLog) {
            return false;
        }
    }

#if PREALLOC_BYTES
    if (bytesWritten + len > PREALLOC_BYTES) {
        LOG_WARN("Log file full, creating new file");
        finish();
        DataSDLogger::setup();
        if (!enableLog) {
            return false;
        }
    }
#endif

    uint32_t written = logFile.write(data, len);
    bytesWritten += written;
    if (written != len) {
        LOG_WARN("Failed write, bytes written", written, "desired length", len);
        return false;
    }

#if FLUSH_MS
    if (flushTimer.check()) {
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

void DataSDLogger::finish() {
    if (!enableLog) {
        return;
    }

    logFile.flush();
    logFile.close();
    bytesWritten = 0;
    enableLog = false;
}
