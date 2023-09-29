#include "ntc.h"

double NTC::temperature() {
    double ntcResistance = pullupResistance / ((ADC_MAX / (double)analogRead(pin)) - 1);
    double lnR = log(ntcResistance);
    return (
        1 / (
            steinhartA + (steinhartB * lnR) + (steinhartC * pow(lnR, 3))
        )
    ) - 273.15;
}

uint16_t NTC::adc() {
    return analogRead(pin);
}