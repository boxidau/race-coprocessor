#include "singletonadc.h"

static ADC* adc;

ADC* SingletonADC::getADC() {
    if (adc == nullptr) {
        adc = new ADC();
    }
    return adc;
}