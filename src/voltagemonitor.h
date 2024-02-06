#pragma once

#include "Arduino.h"
#include "calibratedadc.h"
#include <Metro.h>

const uint16_t UNDERVOLT_3V3  = 3200;
const uint16_t OVERVOLT_3V3   = 3400;
const uint16_t UNDERVOLT_P3V3 = 3000;
const uint16_t OVERVOLT_P3V3  = 3600;
const uint16_t UNDERVOLT_5V   = 4850;
const uint16_t OVERVOLT_5V    = 5100;

// 12V is vehicle voltage
// charging is expected to be 14+ volts
const uint16_t UNDERVOLT_12V = 10500;
const uint16_t OVERVOLT_12V  = 16000;


class VoltageMonitor {

private:
    CalibratedADC sys12v, sys5v, sys3v3, sysp3v3;
    Metro msTick { Metro(1) };
public:
    VoltageMonitor(
        uint sys12vPin,
        uint sys12vADCNum,
        uint sys5vPin,
        uint sys5vADCNum,
        uint sys3v3Pin,
        uint sys3v3ADCNum,
        uint sysp3v3Pin,
        uint sysp3v3ADCNum
    )
        : sys12v { CalibratedADC(sys12vPin, sys12vADCNum) }
        , sys5v { CalibratedADC(sys5vPin, sys5vADCNum) }
        , sys3v3 { CalibratedADC(sys3v3Pin, sys3v3ADCNum) }
        , sysp3v3 { CalibratedADC(sysp3v3Pin, sysp3v3ADCNum) }
    {
    };
    void setup();
    void loop();

    const bool underVoltage();
    const bool overVoltage();

    const uint16_t get3v3MilliVolts();
    const uint16_t getp3v3MilliVolts();
    const uint16_t get5vMilliVolts();
    const uint16_t get12vMilliVolts();
};
