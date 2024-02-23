#include <SD.h>
#include <DebugLog.h>
#include "sdlogger.h"

static bool initialized = false;
static uint32_t retries = 0;

static bool SDLogger::isInitialized() {
    return initialized;
}

static bool SDLogger::ensureInitialized() {
    if (initialized) {
        return true;
    }
    
    if (retries == 3) {
        return false;
    }

    if (!SD.begin(BUILTIN_SDCARD)) {
        LOG_WARN("SD card initialization failed, is a card inserted? Retries:", retries++);
        return false;
    }

    initialized = true;
    return true;
}
