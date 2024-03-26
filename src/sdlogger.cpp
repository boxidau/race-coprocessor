#include <SD.h>
#include <DebugLog.h>
#include "sdlogger.h"

#define RETRY_INTERVAL_MS 5000

static bool initialized = false;
static uint32_t retries = 0;

bool SDLogger::isInitialized() {
    return initialized;
}

void dateTime(uint16_t *date, uint16_t *time)
{
	uint32_t t = now();
	if (t < 315532800) { // before 1980
		*date = 0;
		*time = 0;
	} else {
		DateTimeFields datetime;
		breakTime(t, datetime);
		*date = FS_DATE(datetime.year + 1900, datetime.mon + 1, datetime.mday);
		*time = FS_TIME(datetime.hour, datetime.min, datetime.sec);
	}
}

bool SDLogger::ensureInitialized() {
    if (initialized) {
        return true;
    }
    
    static uint32_t lastRetryTime = millis();
    if (retries == 3 || (retries > 0 && millis() < lastRetryTime + RETRY_INTERVAL_MS)) {
        return false;
    }

    bool ok = SD.begin(BUILTIN_SDCARD);
    //bool ok = SD.sdfs.begin(SdioConfig(FIFO_SDIO));
    if (!ok) {
        LOG_WARN("SD card initialization failed, is a card inserted? Retries:", retries++);
        lastRetryTime = millis();
        return false;
    }

    // set the datetime callback so file timestamps will be on local time rather
    // than UTC
    FsDateTime::setCallback(dateTime);

    initialized = true;
    return true;
}

bool SDLogger::createLogfile(char fileSuffix, FsFile& file) {
    char logDir[16];
    char logFileName[16];
    char fullLogFilePath[32];

    if (!SDLogger::ensureInitialized()) {
        return false;
    }

    static time_t time = now();
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
            return false;
        }
    }

    char suffixStr[2];
    suffixStr[0] = fileSuffix;
    suffixStr[1] = '\0';
    sprintf(logFileName, "%02d%02d%02d%s.csv", hour(time), minute(time), second(time), suffixStr);
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

    sprintf(fullLogFilePath, "%s/%s", logDir, logFileName);
    if (SD.exists(fullLogFilePath)) {
        LOG_WARN("Log file", logFileName, "exists, overwriting");
    }

    file = SD.sdfs.open(fullLogFilePath, O_WRONLY | O_CREAT | O_TRUNC);
    bool success = file.isOpen();
    if (success) {
        LOG_INFO("Created log file", fullLogFilePath);
    } else {
        LOG_WARN("Failed to create log file", fullLogFilePath);
    }

    return success;
}

bool SDLogger::preAlloc(FsFile& file, size_t size) {
    bool didPrealloc = file.preAllocate(size);
    if (!didPrealloc) {
        char errorStr[64];
        sprintf(errorStr, "Preallocing %dMB logfile failed\n", size);
        file.write(errorStr);
        file.flush();
        return false;
    }

    char nameStr[64];
    file.getName(nameStr, sizeof(nameStr));
    LOG_INFO("Preallocing", (float) size / 1000000, "MB for logfile", nameStr, didPrealloc ? "succeeded" : "failed");
    return true;
}
