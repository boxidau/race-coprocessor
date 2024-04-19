#include "ui.h"

#define STARTUP_MILLIS 2000
#define STATUS_LED_BLINK_DURATION_MS 200
#define STATUS_LED_PAUSE_DURATION_MS 1500
const uint PAGES = 10;

Metro pageTurner = Metro(5000);
bool startup = true;

void CoolerUI::setup() {
    display.setup();
    pinMode(uiButtonPin, INPUT);
    displayPageNameUntil = millis() + 1000;
    display.setString("888");        
    display.setLED(ScreenLED::RED, true);
    display.setLED(ScreenLED::YELLOW, true);
    display.setLED(ScreenLED::GREEN, true);
    systemStatusLED.setBoolean(true);
};

bool CoolerUI::shouldDisplayName() {
    return millis() < displayPageNameUntil;
}

void CoolerUI::loop() {
    if (millis() < STARTUP_MILLIS) {
        return;
    }

    if (startup) {
        startup = false;
        display.setLED(ScreenLED::RED, false);
        display.setLED(ScreenLED::YELLOW, false);
        display.setLED(ScreenLED::GREEN, false);
        systemStatusLED.setBoolean(false);
        coolerSystem.getSystemData(rtData);
    }

    if (uiButton.update() && uiButton.fallingEdge()) {
        page = (page + 1) % PAGES;
        displayPageNameUntil = millis() + 1000;
        pageTurnerEnabled = false;
    }

    if (pageTurnerEnabled && pageTurner.check()) {
        page = (page + 1) % PAGES;
        displayPageNameUntil = millis() + 1000; 
    }

    // pulse the green LED with period = flow rate / 4
    uint32_t lastFlowPulse = coolerSystem.lastFlowPulseMicros();
    if (lastFlowPulse != lastFlowPulseDisplayed) {
        if (pulseDivider++ % 2) {
            display.setLED(ScreenLED::GREEN, !display.getLED(ScreenLED::GREEN));
        }
        lastFlowPulseDisplayed = lastFlowPulse;
    }

    // blink error codes to red LED and external status LED
    bool statusLEDState = getSystemStatusLEDState();
    if (!!systemStatusLED.value() != statusLEDState) {
        systemStatusLED.setBoolean(statusLEDState);
        display.setLED(ScreenLED::RED, statusLEDState);
    }

    if (!displayUpdate.check()) {
        return;
    }

    coolerSystem.getSystemData(rtData);

    if (shouldDisplayName()) {
        switch (page) {
            // evap inlet temp
            case 0: display.setString("EIT"); break;
            // evap outlet temp
            case 1: display.setString("EOT"); break;
            // condenser inlet temp
            case 2: display.setString("CIT"); break;
            // condenser outlet temp
            case 3: display.setString("COT"); break;
            // compressor speed
            case 4: display.setString("SPD"); break;
            // compressor current
            case 5: display.setString("CUR"); break;
            // compressor frequency
            case 6: display.setString("FRE"); break;
            // flow rate L/min
            case 7: display.setString("FLO"); break;
            // system pressure
            case 8: display.setString("PRE"); break;
            // error code
            case 9: display.setString("ERR"); break;
        }
        return;
    }
        
    if (page == 8) {
        display.displayError(rtData.fault);
        return;
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
            snprintf(buf, 5, "%4.2f", rtData.condenserInletTemp);
            break;
        case 3:
            snprintf(buf, 5, "%4.2f", rtData.condenserOutletTemp);
            break;
        case 4:
            snprintf(buf, 5, "%d", int(rtData.compressorSpeed * 100));
            break;
        case 5:
            snprintf(buf, 5, "%4.2f", rtData.compressorCurrent);
            break;
        case 6:
            if (rtData.compressorFrequency && roundf(rtData.compressorFrequency) < 1000) {
                snprintf(buf, 5, "%d", (int) roundf(rtData.compressorFrequency));
            } else {
                strcpy(buf, "---");
            }
            break;
        case 7:
            snprintf(buf, 5, "%4.2f", rtData.flowRate / 1000.0);
            break;
        case 8:
            snprintf(buf, 5, "%d", min(rtData.systemPressure, 999));
            break;
    }
    display.setString(buf);
};

bool CoolerUI::getSystemStatusLEDState() {
    uint32_t now;

    switch (rtData.systemStatus) {
        case CoolerSystemStatus::PRECHILL:
        case CoolerSystemStatus::PUMP_LOW:
        case CoolerSystemStatus::PUMP_MEDIUM:
        case CoolerSystemStatus::PUMP_HIGH:
        case CoolerSystemStatus::FLUSH:
            faultBlinks = 0;
            return true;

        case CoolerSystemStatus::REQUIRES_RESET:
            if (!rtData.fault) {
                faultBlinks = 0;
                return false;
            }

            now = millis();
            if (faultBlinks == 0) {
                // entering fault state for the first time
                faultBlinks = getFaultBlinks();
                currentBlink = 1;
                lastBlinkTime = now;
            }
            
            if (currentBlink == faultBlinks + 1 && now < lastBlinkTime + STATUS_LED_PAUSE_DURATION_MS) {
                return false;
            }
            if (currentBlink <= faultBlinks && now < lastBlinkTime + STATUS_LED_BLINK_DURATION_MS) {
                return true;
            }
            if (currentBlink <= faultBlinks && now < lastBlinkTime + 2 * STATUS_LED_BLINK_DURATION_MS) {
                return false;
            }

            currentBlink++;
            lastBlinkTime = now;
            if (currentBlink > faultBlinks + 1) {
                // done blinking fault code, reset so we pull the fault code again
                faultBlinks = 0;
            }

            return false;

        default:
            faultBlinks = 0;
            return false;
    }
}

uint32_t CoolerUI::getFaultBlinks() {
    if (rtData.fault & (byte) SystemFault::LOW_COOLANT) {
        return 1;
    }

    if (rtData.fault & (byte) SystemFault::FLOW_RATE_LOW) {
        return 2;
    }

    if (rtData.fault & (byte) SystemFault::SYSTEM_OVER_PRESSURE) {
        return 3;
    }

    if (rtData.fault & (byte) SystemFault::SYSTEM_UNDERVOLT) {
        return 4;
    }

    if (rtData.fault & (byte) SystemFault::SYSTEM_OVERVOLT) {
        return 5;
    }

    if (rtData.fault & (byte) SystemFault::COMPRESSOR_FAULT) {
        if (rtData.compressorFaultCode == CompressorFaultCode::HIGH_CURRENT) {
            return 6;
        }

        if (rtData.compressorFaultCode == CompressorFaultCode::MOTOR_BLOCKED) {
            return 7;
        }

        if (rtData.compressorFaultCode == CompressorFaultCode::UNDER_VOLTAGE) {
            return 8;
        }

        if (rtData.compressorFaultCode == CompressorFaultCode::FAN_FAILURE) {
            return 9;
        }

        if (rtData.compressorFaultCode == CompressorFaultCode::COMPRESSOR_OFFLINE) {
            return 10;
        }

        if (rtData.compressorFaultCode == CompressorFaultCode::COMPRESSOR_OVERHEAT) {
            return 11;
        }

        if (rtData.compressorFaultCode == CompressorFaultCode::SYSTEM_OVERPRESSURE) {
            return 12;
        }
    }

    return rtData.fault ? 13 : 0;
}
