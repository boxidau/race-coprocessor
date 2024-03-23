/*
Writes CRTD log files for CANBUS logging
https://docs.openvehicles.com/en/latest/crtd
*/
#include "canlogger.h"

#include <DebugLog.h>
#include <Metro.h>
#include <TimeLib.h>
#include "sdlogger.h"
#include "clocktime.h"

static char lineBuffer[512];
static FsFile logFile;
static bool enableLog;
static Metro retrySDTimer = Metro(FLUSH_MS);

bool CANLogger::error = false;

void CANLogger::setup()
{
    if (!SDLogger::ensureInitialized()) {
        return;
    }

    char logDir[16];
    char logFileName[16];
    char fullLogFilePath[32];

    time_t time = now();
    sprintf(logDir, "%d%02d%02d", year(time), month(time), day(time));
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

    sprintf(logFileName, "%02d%02d%02d.csv", hour(time), minute(time), second(time));
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
    #if PREALLOC_MB
        LOG_INFO("Preallocing", PREALLOC_MB, "MB logfile,", logFile.preAllocate(PREALLOC_MB * 1000000) ? "success" : "failure");
    #endif

    logFile.write("time,evapInletTemp,evapInletA10Temp,evapOutletTemp,condInletTemp,condOutletTemp,ambientTemp,flowRate,pressure,coolantLevel,12v,5v,3v3,p3v3,coolingPower,switchPos,switchADC,status,systemEnable,chillerPumpEnable,coolshirtEnable,compressorSpeed,underTempCutoff,systemFault,compressorFault,slowLoopTime\n");
    enableLog = true;
}

void CANLogger::logComment(const String line)
{
    char strBuf[241];
    line.toCharArray(strBuf, 240);

    snprintf(lineBuffer, 254, "%0.3f CXX %s", ClockTime::secSinceEpoch(), strBuf);
    write();
}

void CANLogger::stringify(char output[255], const CAN_message_t &message, bool rx) {
    sprintf(output, "%0.3f %d%s%s %lu ",
            ClockTime::secSinceEpoch(),
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

bool CANLogger::logMessage(StringFormatCSV& format)
{
    return write(format);
}

bool CANLogger::write(StringFormatCSV& format)
{
    bool tick = retrySDTimer.check();
    if (!enableLog)
    {
        if (tick)
        {
            CANLogger::setup();
        }
        return false;
    }

    const char* buffer = format.finish();
    uint32_t length = format.length();
    uint32_t m = logFile.write(buffer, length);
    if (m != length)
    {
        LOG_INFO("Failed write, bytes written", m, "desired length", length);
        enableLog = false;
        error = true;
        return false;
    }

    error = false;
    
    #if FLUSH_MS
        if (tick)
        {
            logFile.flush();
            return true;
        }
    #endif

    return false;
}