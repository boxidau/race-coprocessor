/*
Writes CRTD log files for CANBUS logging
https://docs.openvehicles.com/en/latest/crtd
*/
#include "canlogger.h"

#include "clocktime.h"

#include <DebugLog.h>
#include <Metro.h>
#include <TimeLib.h>

static char lineBuffer[512];
static int linesWritten;
static File logFile;
static bool enableLog;
static int retries;
static Metro retrySDTimer = Metro(5000);

bool CANLogger::error = false;

void CANLogger::setup()
{
    return;
    if (retries++ == 3) {
        return;
    }

    linesWritten = 0;

    LOG_DEBUG("Initializing SD card");
    if (!SD.begin(BUILTIN_SDCARD))
    {
        LOG_WARN("SD card initialization failed, is a card inserted? Retries:", retries);
        return;
    }

    LOG_DEBUG("An SD card is present");

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

    logFile = SD.open(fullLogFilePath, FILE_WRITE);
    logFile.println("time,evapInletTemp,evapCalculatedAvg,evapOutletTemp,condInletTemp,condOutletTemp,ambientTemp,flowRate,pressure,12v,5v,3v3,p3v3,coolingPower,switchPos,status,systemEnable,chillerPumpEnable,coolshirtPWM,compressorSpeed,underTempCutoff,systemFault,slowLoopTime,didSDFlush");
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

void CANLogger::logMessage(const char* message)
{
    strcpy(lineBuffer, message);
    write();
}

void CANLogger::write()
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
    sprintf(lineBuffer + strlen(lineBuffer), ",%u", tick);
    uint32_t m1 = micros();
    static uint32_t n = 0;
    uint32_t m = 0;
    if ((m = logFile.println(lineBuffer)) != strlen(lineBuffer) + 2)
    {
        enableLog = false;
        error = true;
        return;
    }
    uint32_t m2 = micros();
    n += m;
    //LOG_INFO("write", n, " cumulative bytes", m2-m1, "us");

    ++linesWritten;
    error = false;
    
    if (tick)
    {
        LOG_DEBUG("LOG FLUSH, written lines: ", linesWritten );
        m1 = micros();
        logFile.flush();
        m2 = micros();
        n = 0;
        //LOG_INFO("flush", m2-m1);
    }
}