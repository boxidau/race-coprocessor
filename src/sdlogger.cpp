#include <SD.h>
#include <DebugLog.h>
#include "sdlogger.h"

#define RETRY_INTERVAL_MS 5000

static bool initialized = false;
static uint32_t retries = 0;

bool SDLogger::isInitialized() {
    return initialized;
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

    initialized = true;
    return true;
}
