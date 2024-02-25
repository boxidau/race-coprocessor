#include "ntclogger.h"

#include <DebugLog.h>
#include <Metro.h>
#include <TimeLib.h>
#include <SD.h>

#include "clocktime.h"
#include "sdlogger.h"
#include "stringformat.h"

void NTCLogger::ensureSetup()
{
    if (enableLog) {
        return;
    }

    if (!SDLogger::ensureInitialized()) {
        return;
    }

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
    logFile.print("time,ntc1,ntc1avg,ntc2,ntc2avg\n");
    enableLog = true;
}

void NTCLogger::logSamples(uint32_t time, uint16_t ntc1, uint16_t ntc1avg, uint16_t ntc2, uint16_t ntc2avg) {
    if (!enableLog) {
        return;
    }

    NTCData data;
    data.time = time;
    data.ntc1 = ntc1;
    data.ntc1avg = ntc1avg;
    data.ntc2 = ntc2;
    data.ntc2avg = ntc2avg;
    ntcData.push_back(data);

    if (ntcData.full()) {
        flush();
    }
}

void NTCLogger::flush()
{
    if (!enableLog) {
        return;
    }

    unsigned long start = micros();
    for (size_t i = 0; i < ntcData.size(); i++) {
        NTCData& data = ntcData[i];

        char message[128];
        StringFormatCSV format(message, sizeof(message));
        format.formatUnsignedInt(data.time);
        format.formatUnsignedInt(data.ntc1);
        format.formatUnsignedInt(data.ntc1avg);
        format.formatUnsignedInt(data.ntc2);
        format.formatUnsignedInt(data.ntc2avg);

        logFile.write(format.finish(), format.length());
    }
    unsigned long end = micros();

    ntcData.clear();
    logFile.flush();
    LOG_INFO("NTC LOG FLUSHED, print ", end - start, ", flush ", micros() - end);
}
