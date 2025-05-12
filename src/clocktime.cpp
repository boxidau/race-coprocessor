#include "clocktime.h"

#include <DebugLog.h>
#include <Timezone.h>

static uint32_t epoch = 0;

TimeChangeRule PDT = { "PDT", Second, Sun, Mar, 2, -420 };    //Daylight time = UTC - 7 hours
TimeChangeRule PST = { "PST", First, Sun, Nov, 2, -480 };     //Standard time = UTC - 8 hours
Timezone Pacific(PDT, PST);

struct AstronomicalTimeRange {
    uint8_t startHour;
    uint8_t startMinute;
    uint8_t endHour;
    uint8_t endMinute;
};

// https://www.timeanddate.com/sun/usa/san-francisco
// SF, May 30
static const AstronomicalTimeRange civilTwilightMorning {5, 19, 5, 50};
static const AstronomicalTimeRange civilTwilightEvening {20, 24, 20, 55};

static const uint8_t dayDimmer = 100; // %
static const uint8_t nightDimmer = 25; // %

static int timeOffsetFrom(time_t now, int h, int m) {
    return (hour(now) - h) * 60 + minute(now) - m;
}

static time_t getTeensy3Time()
{
    // this is called to fetch the RTC and sync to CPU time.
    // apply timezone offset here so CPU time is always local,
    // and RTC time (set when serial is connected) is UTC.
    // having CPU time be local is important for filesystem
    // timestamps.
    return Pacific.toLocal(Teensy3Clock.get());
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
        time_t n = now();
        LOG_INFO("RTC has set the local system time to ", year(n), hour(n), minute(n), second(n));
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

uint8_t ClockTime::getDimmerBrightnessPercent() {
    // during nighttime, use 25%
    // linearly increase to 100% during civil twilight morning
    // stay at 100% during daytime
    // transition to 25% during civil twilight evening
    time_t n = now();
    int fromStart = timeOffsetFrom(n, civilTwilightMorning.startHour, civilTwilightMorning.startMinute);
    if (fromStart < 0) {
        return nightDimmer;
    }
    int fromEnd = timeOffsetFrom(n, civilTwilightMorning.endHour, civilTwilightMorning.endMinute);
    int duration = fromStart - fromEnd;
    if (fromEnd < 0) {
        return fromStart * (dayDimmer - nightDimmer) / duration + nightDimmer;
    }

    fromStart = timeOffsetFrom(n, civilTwilightEvening.startHour, civilTwilightEvening.startMinute);
    if (fromStart < 0) {
        return dayDimmer;
    }
    fromEnd = timeOffsetFrom(n, civilTwilightEvening.endHour, civilTwilightEvening.endMinute);
    duration = fromStart - fromEnd;
    if (fromEnd < 0) {
        return fromStart * (nightDimmer - dayDimmer) / duration + dayDimmer;
    }

    return nightDimmer;
}
