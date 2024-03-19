#include "clocktime.h"

#include <DebugLog.h>
#include <Timezone.h>

static uint32_t epoch = 0;

TimeChangeRule PDT = { "PDT", Second, Sun, Mar, 2, -420 };    //Daylight time = UTC - 7 hours
TimeChangeRule PST = { "PST", First, Sun, Nov, 2, -480 };     //Standard time = UTC - 8 hours
Timezone Pacific(PDT, PST);

static time_t getTeensy3Time()
{
    return Teensy3Clock.get();
}

void ClockTime::setup()
{
    setSyncProvider(getTeensy3Time);
    if (timeStatus() != timeSet)
    {
        LOG_ERROR("Unable to sync with the RTC");
    }
    else
    {
        LOG_INFO("RTC has set the system time");
    }
}

void ClockTime::setEpoch() {
    epoch = millis();
}

uint32_t ClockTime::millisSinceEpoch()
{
    return epoch ? millis() - epoch : 0;
}

double ClockTime::secSinceEpoch()
{
    return epoch ? (double)(millis() - epoch) / 1000 : 0;
}

time_t ClockTime::getLocalTime() {
    time_t time = now();
    return Pacific.toLocal(time);
}
