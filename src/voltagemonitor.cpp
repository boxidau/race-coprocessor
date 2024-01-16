#include "voltagemonitor.h"

Metro msTick = Metro(1);

void VoltageMonitor::setup() {
    sys12v.setup();
    sys12v.setCalibration(0, 0, 39777, 12000, false);
    sys5v.setup();
    sys5v.setCalibration(0, 0, 56740, 5000, false);
    sys3v3.setup();
    sys3v3.setCalibration(0, 0, 54592, 3300, false);
};

void VoltageMonitor::loop() {
    if (msTick.check()) {
        sys12v.loop();
        sys5v.loop();
        sys3v3.loop();
        if (startup && millis() > 1000) {
            startup = false;
        }
    }
};

const bool VoltageMonitor::underVoltage() {
    if (startup) return false;
    return (get3v3MilliVolts() < UNDERVOLT_3V3)
        || (get5vMilliVolts() < UNDERVOLT_5V)
        || (get12vMilliVolts() < UNDERVOLT_12V);
};
const bool VoltageMonitor::overVoltage() {
    if (startup) return false;
    return (get3v3MilliVolts() > OVERVOLT_3V3)
        || (get5vMilliVolts() > OVERVOLT_5V)
        || (get12vMilliVolts() > OVERVOLT_12V);
};

const uint16_t VoltageMonitor::get3v3MilliVolts() {
    return sys3v3.calibratedValue();
};

const uint16_t VoltageMonitor::get5vMilliVolts() {
    return sys5v.calibratedValue();
};

const uint16_t VoltageMonitor::get12vMilliVolts() {
    // Serial.println(sys12v.calibratedValue());
    return sys12v.calibratedValue();
};
