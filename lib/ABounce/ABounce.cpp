
// Please read Bounce.h for information about the liscence and authors

#include <Arduino.h>
#include "ABounce.h"


ABounce::ABounce(uint8_t pin,unsigned long interval_millis, uint16_t threshold)
{
	interval(interval_millis);
	previous_millis = millis();
	state = analogRead(pin) > threshold;
    this->pin = pin;
    this->threshold = threshold;
}


void ABounce::interval(unsigned long interval_millis)
{
  this->interval_millis = interval_millis;
  this->rebounce_millis = 0;
}

void ABounce::rebounce(unsigned long interval)
{
	 this->rebounce_millis = interval;
}



int ABounce::update()
{
	if ( debounce() ) {
        rebounce(0);
        return stateChanged = 1;
    }

     // We need to rebounce, so simulate a state change
     
	if ( rebounce_millis && (millis() - previous_millis >= rebounce_millis) ) {
        previous_millis = millis();
		 rebounce(0);
		 return stateChanged = 1;
	}

	return stateChanged = 0;
}


unsigned long ABounce::duration()
{
  return millis() - previous_millis;
}


int ABounce::read()
{
	return (int)state;
}


// Protected: debounces the pin
int ABounce::debounce() {
	
	uint8_t newState = analogRead(pin) > threshold;
	if (state != newState ) {
  		if (millis() - previous_millis >= interval_millis) {
  			previous_millis = millis();
  			state = newState;
  			return 1;
	}
  }
  
  return 0;
	
}

// The risingEdge method is true for one scan after the de-bounced input goes from off-to-on.
bool  ABounce::risingEdge() { return stateChanged && state; }
// The fallingEdge  method it true for one scan after the de-bounced input goes from on-to-off. 
bool  ABounce::fallingEdge() { return stateChanged && !state; }

