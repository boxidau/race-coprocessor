/*
Writes CRTD log files for CANBUS logging
https://docs.openvehicles.com/en/latest/crtd
*/
#include "canlogger.h"

#include "clocktime.h"

#include <DebugLog.h>
#include <Metro.h>
#include <TimeLib.h>
#include "sdlogger.h"

static char lineBuffer[512];
static int linesWritten;
static FsFile logFile;
static bool enableLog;
static Metro retrySDTimer = Metro(FLUSH_MS);

bool CANLogger::error = false;

void CANLogger::setup()
{
    linesWritten = 0;
    if (!SDLogger::ensureInitialized()) {
        return;
    }

    char logDir[9];
    char logFileName[13];
    char fullLogFilePath[19];

    sprintf(logDir, "%d%02d%02d", year(), month(), day());
    if (year() < 1980) // rtc is not set
    {
        sprintf(logDir, "%s", "NODATE");
    }
    if (!SD.exists(logDir))
    {
        if (!SD.mkdir(logDir))
        {
            LOG_ERROR("Unable to make log directory", logDir);
            return;
        }
    }
    LOG_INFO("Log directory OK", logDir);


    sprintf(logFileName, "%02d%02d%02d.csv", hour(), minute(), second());
    if (year() < 1980) // rtc is not set
    {
        File noDateDirectory = SD.open(logDir);
        File existingLogFile;
        int maxLog = 0;
        while (existingLogFile = noDateDirectory.openNextFile()) {
            int curLog = 0;
            if(sscanf(existingLogFile.name(), "%d.csv", &curLog) && curLog > maxLog) {
                maxLog = curLog;
            }
        }
        noDateDirectory.close();
        sprintf(logFileName, "%08d.csv", maxLog + 1);   
    }

    LOG_INFO("Log directory", logDir);
    LOG_INFO("Log file name", logFileName);
    sprintf(fullLogFilePath, "%s/%s", logDir, logFileName);
    LOG_INFO("Log file full path", fullLogFilePath);

    if (SD.exists(fullLogFilePath))
    {
        LOG_ERROR("Log file already exists, overwriting");
    }

    logFile = SD.sdfs.open(fullLogFilePath, O_WRONLY | O_CREAT | O_TRUNC);
    //logFile = SD.open(fullLogFilePath, FILE_WRITE);
    #if PREALLOC_BYTES
        LOG_INFO("preallocing:", logFile.preAllocate(PREALLOC_BYTES) ? "success" : "failure");
    #endif

    logFile.write("time,evapInletTemp,evapOutletTemp,condInletTemp,condOutletTemp,ambientTemp,flowRate,pressure,12v,5v,3v3,p3v3,coolingPower,switchPos,status,systemEnable,chillerPumpEnable,coolshirtPWM,compressorSpeed,underTempCutoff,systemFault,slowLoopTime,didSDFlush,sdIsBusy\n");
    enableLog = true;
}

void CANLogger::logComment(const String line)
{
    char strBuf[241];
    line.toCharArray(strBuf, 240);

    snprintf(lineBuffer, 254, "%0.3f CXX %s", ClockTime::getMillisTime(), strBuf);
    write();
}

void CANLogger::stringify(char output[255], const CAN_message_t &message, bool rx) {
    sprintf(output, "%0.3f %d%s%s %lu ",
            ClockTime::getMillisTime(),
            1, // hardcode canbus 1 for now
            rx ? "R" : "T",
            message.ext ? "29" : "11",
            message.id
    );

    for (int i = 0; i < message.len; i++) {
        char hexBuf[3];
        sprintf(hexBuf, "%02X",  message.buf[i]);
        strcat(output, hexBuf);
        if (i != message.len-1) {
            strcat(output, " ");
        }
    }
}

void CANLogger::logCANMessage(const CAN_message_t &message, bool rx)
{
    stringify(lineBuffer, message, rx);
    write();
}

void CANLogger::logMessage(StringFormatCSV& format)
{
    write(format);
}

void CANLogger::write(StringFormatCSV& format)
{
    bool tick = retrySDTimer.check();
    if (!enableLog)
    {
        if (tick)
        {
            CANLogger::setup();
        }
        return;
    }

    //LOG_DEBUG("LOG WRITE:", + lineBuffer);
    format.formatBool(tick);
    format.formatBool(logFile.isBusy());
    const char* buffer = format.finish();
    uint32_t length = format.length();
    uint32_t m1 = micros();
    static uint32_t n = 0;
    uint32_t m = 0;
    m = logFile.write(buffer, length);
    if (m != length)
    {
        LOG_INFO("failed write, bytes written", m, "desired length", length);
        enableLog = false;
        error = true;
        return;
    }
    uint32_t m2 = micros();
    n += m;
    //LOG_INFO("write", n, "cumulative bytes", m2-m1, "us, was busy", isbusy);

    ++linesWritten;
    error = false;
    
    #if FLUSH_MS
        if (tick)
        {
            LOG_DEBUG("LOG FLUSH, written lines: ", linesWritten );
            m1 = micros();
            logFile.flush();
            m2 = micros();
            LOG_INFO("flush", m2-m1, "us, bytes", n);
            LOG_INFO("buffer", buffer);
            n = 0;
        }
    #endif
}