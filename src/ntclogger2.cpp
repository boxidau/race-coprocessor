#include "ntclogger2.h"

#include "clocktime.h"

#include <DebugLog.h>
#include <Metro.h>
#include <TimeLib.h>
#include <SdFat.h>

SdFat32 sd;
File32 file;

#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else   // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

void NTCLogger2::setup()
{
    if (!sd.begin(SdioConfig(FIFO_SDIO))) {
        if (sd.sdErrorCode()) {
            if (sd.sdErrorCode() == SD_CARD_ERROR_ACMD41) {
            Serial.println("Try power cycling the SD card.");
            }
            printSdErrorSymbol(&Serial, sd.sdErrorCode());
            Serial.print(", ErrorData: 0X");
            Serial.println(sd.sdErrorData(), HEX);
        }
    }
    Serial.println("\nDedicated SPI mode.");

    lineBuf = new char[LINEBUF_SIZE];

    LOG_DEBUG("Initializing SD card");
/*
    if (!.begin(BUILTIN_SDCARD))
    {
        LOG_WARN("SD card initialization failed, is a card inserted?");
        return;
    }
*/
    LOG_DEBUG("An SD card is present");

    char logDir[10];
    char logFileName[20];
    char fullLogFilePath[30];

    sprintf(logDir, "%d%02d%02d", year(), month(), day());
    if (year() < 1980) // rtc is not set
    {
        sprintf(logDir, "%s", "NODATE");
    }
    if (!sd.exists(logDir))
    {
        if (!sd.mkdir(logDir))
        {
            LOG_ERROR("Unable to make log directory", logDir);
            return;
        }
    }
    LOG_INFO("Log directory OK", logDir);


    sprintf(logFileName, "n%02d%02d%02d.csv", hour(), minute(), second());
    if (year() < 1980) // rtc is not set
    {
        File32 noDateDirectory = sd.open(logDir);
        File32 existingLogFile;
        int maxLog = 0;
        while (existingLogFile = noDateDirectory.openNextFile()) {
            int curLog = 0;
            char existingLogFileName[32];
            existingLogFile.getName(existingLogFileName, 32);
            if(sscanf(existingLogFileName, "n%d.csv", &curLog) && curLog > maxLog) {
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


    if (sd.exists(fullLogFilePath))
    {
        LOG_ERROR("Log file already exists, overwriting");
    }

    logFile = sd.open(fullLogFilePath, FILE_WRITE);
    enableLog = true;
}

void NTCLogger2::logSamples(uint16_t ntc1, uint16_t ntc2, uint16_t ntcDifferential, uint16_t ntc3, uint16_t ntc4, uint16_t ambient) {
    // 4 digits + 5 * 5 + commas + newline = 35 chars per line
    unsigned long time = micros();
    prevTime = started ? prevTime : time;
    unsigned long duration = time >= prevTime ? time - prevTime : time + (UINT32_MAX - prevTime);
    time64 += duration;
    prevTime = time;
    started = true;

    if (!enableLog) {
        return;
    }
    unsigned long micro = micros();
    //bufWritten += snprintf(lineBuf + bufWritten, 64, "%lu,%u,%u,%u,%u,%u,%u\n", (long unsigned int) data.time, data.ntc1, data.ntc2, data.ntcDifferential, data.ntc3, data.ntc4, data.ambient);
    bufWritten += snprintf(lineBuf + bufWritten, 64, "%llu,%u\n", time64 / 1000, ambient);
    int fileWritten = logFile.print(lineBuf);
    //LOG_INFO("written line ", micros() - micro);
 
    if (bufWritten > LINEBUF_SIZE - 64) {
        flush();
    }
}

void NTCLogger2::flush()
{
    if (!enableLog) {
        return;
    }

    //LOG_DEBUG("NTC LOG FLUSH, writing lines: ", ntcData.size());
/*
    int bufWritten = 0;
    int fileWritten = 0;
    unsigned long micro = micros();
    for (uint i = 0; i < ntcData.size(); i++) {
        NTCData& data = ntcData[i];
        //bufWritten += snprintf(lineBuf + bufWritten, 64, "%lu,%u,%u,%u,%u,%u,%u\n", (long unsigned int) data.time, data.ntc1, data.ntc2, data.ntcDifferential, data.ntc3, data.ntc4, data.ambient);
        bufWritten += snprintf(lineBuf + bufWritten, 64, "%lu,%u\n", (long unsigned int) data.time, data.ambient);
        if (bufWritten > LINEBUF_SIZE - 64) {
            //fileWritten = logFile.print(lineBuf);
            if (false && fileWritten != bufWritten)
            {
                enableLog = false;
                LOG_ERROR("NTC LOG FLUSH ERROR, buffer bytes written: ", bufWritten, ", file bytes written: ", fileWritten);
                return;
            }

            bufWritten = 0;
        }
    }
*/
    unsigned long micro = micros();
    int fileWritten = logFile.print(lineBuf);
    if (false && fileWritten != bufWritten)
    {
        enableLog = false;
        LOG_ERROR("NTC LOG FLUSH ERROR, buffer bytes written: ", bufWritten, ", file bytes written: ", fileWritten);
        return;
    }

    bufWritten = 0;

    unsigned long doneprint = micros();

    //ntcData.clear();
    logFile.flush();
    LOG_INFO("NTC LOG FLUSHED, print ", doneprint - micro, ", flush ", micros() - doneprint);
}
