#pragma once

#include "Arduino.h"
#include "calibratedadc.h"
#include <Metro.h>

const uint16_t UNDERVOLT_3V3 = 3200;
const uint16_t OVERVOLT_3V3  = 3400;
const uint16_t UNDERVOLT_5V  = 4850;
const uint16_t OVERVOLT_5V   = 5100;

// 12V is vehicle voltage
// charging is expected to be 14+ volts
const uint16_t UNDERVOLT_12V = 11500;
const uint16_t OVERVOLT_12V  = 15000;


class VoltageMonitor {

private:
    CalibratedADC sys12v, sys5v, sys3v3;
    Metro msTick { Metro(1) };
    bool startup { true };
public:
    VoltageMonitor(
        uint sys12vPin,
        uint sys5vPin,
        uint sys3v3Pin
    )
        : sys12v { CalibratedADC(sys12vPin) }
        , sys5v { CalibratedADC(sys5vPin) }
        , sys3v3 { CalibratedADC(sys3v3Pin) }
    {
    };
    void setup();
    void loop();

    const bool underVoltage();
    const bool overVoltage();

    const uint16_t get3v3MilliVolts();
    const uint16_t get5vMilliVolts();
    const uint16_t get12vMilliVolts();
};