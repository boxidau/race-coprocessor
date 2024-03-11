#include <SD.h>
#include <DebugLog.h>
#include "sdlogger.h"

static bool initialized = false;
static uint32_t retries = 0;

bool SDLogger::isInitialized() {
    return initialized;
}

bool SDLogger::ensureInitialized() {
    if (initialized) {
        return true;
    }
    
    if (retries == 3) {
        return false;
    }

    bool ok = SD.begin(BUILTIN_SDCARD);
    //bool ok = SD.sdfs.begin(SdioConfig(FIFO_SDIO));
    //bool ok = SD.sdfs.begin(SdioConfig(DMA_SDIO));
    // CS= 10 or 4?
    //bool ok = SD.sdfs.begin(SdSpiConfig(SS, DEDICATED_SPI, SD_SCK_MHZ(50)));
    if (!ok) {
        LOG_WARN("SD card initialization failed, is a card inserted? Retries:", retries++);
        return false;
    }

    initialized = true;
    return true;
}
