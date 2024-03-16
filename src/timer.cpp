
#include "timer.h"
#include <Arduino.h>

MetroTimer::MetroTimer(uint32_t interval_millis)
{
    intervalMillis = interval_millis;
    reset();
}

bool MetroTimer::check()
{
    if (firstCheck) {
        firstCheck = false;
        return true;
    }

    int32_t delta = millis() - previousMillis;
    if (delta >= (int32_t) intervalMillis) {
        // if > N.5x the interval has elapsed, skip the next N events. this prevents multiple
        // rapidfire events to play catchup.
        uint32_t intervalAdd = delta / intervalMillis + (2 * (delta % intervalMillis) >= intervalMillis);
    		previousMillis += intervalAdd * intervalMillis;
        return true;
	  }
  
    return false;
}

void MetroTimer::reset() 
{
    previousMillis = millis();
}
