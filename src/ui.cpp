#include "ui.h"

#define STARTUP_MILLIS 2000
const uint PAGES = 9;

Metro pageTurner = Metro(10000);
bool startup = true;

void CoolerUI::setup() {
    display.setup();
    pinMode(uiButtonPin, INPUT);
    displayPageNameUntil = millis() + 1000;
};

bool CoolerUI::shouldDisplayName() {
    return millis() < displayPageNameUntil;
}

void CoolerUI::loop() {
    if (millis() < STARTUP_MILLIS) {
        display.setString("888");
        display.setLED(ScreenLED::RED, true);
        display.setLED(ScreenLED::YELLOW, true);
        display.setLED(ScreenLED::GREEN, true);
        return;
    }
    if (startup) {
        startup = false;
        display.setLED(ScreenLED::RED, false);
        display.setLED(ScreenLED::YELLOW, false);
        display.setLED(ScreenLED::GREEN, false);
    }

    display.setLED(ScreenLED::RED, rtData.fault);

    if (
        (uiButton.update() && uiButton.fallingEdge())
    ) {
        page = (page + 1) % PAGES;
        displayPageNameUntil = millis() + 1000;
        pageTurnerEnabled = false;
    }

    if (pageTurnerEnabled && pageTurner.check()) {
        page = (page + 1) % PAGES;
        displayPageNameUntil = millis() + 1000; 
    }

    // pulse the green LED with period = flow rate / 4
    unsigned long lastFlowPulse = coolerSystem.lastFlowPulseMicros();
    if (lastFlowPulse != lastFlowPulseDisplayed) {
        if (pulseDivider++ % 2) {
            display.setLED(ScreenLED::GREEN, !display.getLED(ScreenLED::GREEN));
        }
        lastFlowPulseDisplayed = lastFlowPulse;
    }

    if (!displayUpdate.check()) {
        return;
    }

    coolerSystem.getSystemData(rtData);

    if (shouldDisplayName()) {
        switch (page) {
            // evap inlet temp
            case 0: return display.setString("EIT");
            // evap outlet temp
            case 1: return display.setString("EOT");
            // pwm
            case 2: return display.setString("PWM");
            // flow rate L/min
            case 3: return display.setString("FLO");
            // system pressure
            case 4: return display.setString("PRE");
            // error code
            case 5: return display.setString("ERR");
            // condenser inlet temp
            case 6: return display.setString("CIT");
            // condenser outlet temp
            case 7: return display.setString("COT");
            // ambient temp
            case 8: return display.setString("AMT");
        }
    }
        
    if (page == 5) {
        return display.displayError(rtData.fault);
    }

    // output buffer size is 5 screen size is 3, why?
    // decimal point chars are collapsed to prior char
    // so they don't count and c_strs are null terminated
    char buf[5] = {0, 0, 0, 0, 0};
    switch (page) {
        case 0:
            snprintf(buf, 5, "%4.2f", rtData.evaporatorInletTemp);
            break;
        case 1:
            snprintf(buf, 5, "%4.2f", rtData.evaporatorOutletTemp);
            break;
        case 2:
            snprintf(buf, 5, "%d", int(rtData.compressorSpeed * 100));
            break;
        case 3:
            snprintf(buf, 5, "%4.2f", rtData.flowRate / 1000.0);
            break;
        case 4:
            snprintf(buf, 5, "%d", (uint)rtData.systemPressure);
            break;
        case 6:
            snprintf(buf, 5, "%4.2f", rtData.condenserInletTemp);
            break;
        case 7:
            snprintf(buf, 5, "%4.2f", rtData.condenserOutletTemp);
            break;
        case 8:
            snprintf(buf, 5, "%4.2f", rtData.ambientTemp);
            break;
    }
    display.setString(buf);
};


const CoolerSystemData& CoolerUI::getCSData() {
    return rtData;
};