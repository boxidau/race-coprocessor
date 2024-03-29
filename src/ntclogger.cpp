#include "ntclogger.h"

#include "clocktime.h"

#include <DebugLog.h>
#include <Metro.h>
#include <TimeLib.h>
#include <SD.h>

void NTCLogger::setup()
{
    lineBuf = new char[LINEBUF_SIZE];

    LOG_DEBUG("Initializing SD card");
    if (!SD.begin(BUILTIN_SDCARD))
    {
        LOG_WARN("SD card initialization failed, is a card inserted?");
        return;
    }

    LOG_DEBUG("An SD card is present");

    char logDir[10];
    char logFileName[20];
    char fullLogFilePath[30];

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


    sprintf(logFileName, "n%02d%02d%02d.csv", hour(), minute(), second());
    if (year() < 1980) // rtc is not set
    {
        File noDateDirectory = SD.open(logDir);
        File existingLogFile;
        int maxLog = 0;
        while (existingLogFile = noDateDirectory.openNextFile()) {
            int curLog = 0;
            if(sscanf(existingLogFile.name(), "ntc_%d.csv", &curLog) && curLog > maxLog) {
                maxLog = curLog;
            }
        }
        noDateDirectory.close();
        sprintf(logFileName, "n%05d.csv", maxLog + 1);   
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
    enableLog = true;
}

void NTCLogger::logSamples(uint16_t ntc1, uint16_t ntc2, uint16_t ntcDifferential, uint16_t ntc3, uint16_t ntc4, uint16_t ambient) {
    if (!enableLog) {
        return;
    }

    // 4 digits + 5 * 5 + commas + newline = 35 chars per line
    unsigned long time = micros();
    prevTime = started ? prevTime : time;
    unsigned long duration = time >= prevTime ? time - prevTime : time + (UINT32_MAX - prevTime);
    time64 += duration;
    prevTime = time;
    started = true;

    NTCData data;
    data.time = time64 / 1000;
    data.ntc1 = ntc1;
    data.ntc2 = ntc2;
    data.ntcDifferential = ntcDifferential;
    data.ntc3 = ntc3;
    data.ntc4 = ntc4;
    data.ambient = ambient;
    ntcData.push_back(data);
    if (ntcData.size() == ntcData.max_size()) {
        flush();
    }
}

void NTCLogger::flush()
{
    if (!enableLog) {
        return;
    }

    LOG_DEBUG("NTC LOG FLUSH, writing lines: ", ntcData.size());

    int bufWritten = 0;
    int fileWritten = 0;
    for (uint i = 0; i < ntcData.size(); i++) {
        NTCData& data = ntcData[i];
        bufWritten += snprintf(lineBuf + bufWritten, 64, "%lu,%u,%u,%u,%u,%u,%u\n", (long unsigned int) data.time, data.ntc1, data.ntc2, data.ntcDifferential, data.ntc3, data.ntc4, data.ambient);
        if (bufWritten > LINEBUF_SIZE - 64) {
            fileWritten = logFile.print(lineBuf);
            if (fileWritten != bufWritten)
            {
                enableLog = false;
                LOG_ERROR("NTC LOG FLUSH ERROR, buffer bytes written: ", bufWritten, ", file bytes written: ", fileWritten);
                return;
            }

            bufWritten = 0;
        }
    }

    snprintf(lineBuf, 10, ",,,,,,\n");
    logFile.print(lineBuf);
    ntcData.clear();
    logFile.flush();
    LOG_INFO("NTC LOG FLUSHED");
}
