#include <TimeLib.h>

class ClockTime
{
public:
    static void setup();
    static void setEpoch();
    static uint32_t millisSinceEpoch();
    static double secSinceEpoch();
    static time_t getLocalTime();
};
