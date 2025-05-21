#include "ui.h"
#include "clocktime.h"

#define STARTUP_MILLIS 2000
#define STATUS_LED_BLINK_DURATION_MS 200
#define STATUS_LED_PAUSE_DURATION_MS 1500
#define SYSTEM_STATUS_LED_BASE_BRIGHTNESS 25 // %
const uint PAGES = 10;

Metro pageTurner = Metro(5000);
bool startup = true;

#define INT_TO_CHAR(in) (0x30 + (in))

// format an unsigned int into 3 chars, right justified
void uito3a(char* out, uint32_t i) {
    if (i >= 1000) {
        // overflow
        out[0] = '-';
        out[1] = '-';
        out[2] = '-';
        out[3] = '\0';
        return;
    }

    out[3] = '\0';
    out[2] = INT_TO_CHAR(i % 10);
    i /= 10;

    if (i != 0) {
        out[1] = INT_TO_CHAR(i % 10);
    } else {
        out[1] = ' ';
    }
    i /= 10;

    if (i != 0) {
        out[0] = INT_TO_CHAR(i);
    } else {
        out[0] = ' ';
    }
}

// format a float into 4 chars, including decimal point.
// this will mean 3 significant figures if positive, 2 if negative.
void dto3a(char* out, float in) {
    if (in >= 1000 || in <= -100) {
        // overflow
        out[0] = '-';
        out[1] = '-';
        out[2] = '-';
        out[3] = '\0';
        return;
    }

    out[4] = '\0';

    uint32_t index = 0;
    bool isNegative = in < 0;
    if (isNegative) {
        out[index++] = '-';
        in = -in;
    }

    uint32_t decimalPointIndex;
    if (in >= 100) {
        decimalPointIndex = 2;
    } else if (in >= 10) {
        decimalPointIndex = 1;
        in *= 10;
    } else {
        decimalPointIndex = 0;
        in *= 100;
    }

    uint32_t val = (uint32_t) in;
    out[index++] = INT_TO_CHAR(val / 100);
    if (decimalPointIndex == 0) {
        out[index++] = '.';
    }
    out[index++] = INT_TO_CHAR((val / 10) % 10);
    if (decimalPointIndex == 1) {
        out[index++] = '.';
    }
    if (isNegative) {
        return;
    }
    out[index++] = INT_TO_CHAR(val % 10);
    if (decimalPointIndex == 2) {
        out[index++] = '.';
    }
}

void CoolerUI::setup() {
    display.setup();
    pinMode(uiButtonPin, INPUT);
    displayPageNameUntil = millis() + 1000;
    display.setString("888");        
    display.setLED(ScreenLED::RED, true);
    display.setLED(ScreenLED::YELLOW, true);
    display.setLED(ScreenLED::GREEN, true);
    systemStatusLED.setup();
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

    coolerSystem.getSystemData(rtData);

    // pulse the green LED with period = flow rate / 4
    if (rtData.flowRate > 0) {
        uint32_t lastFlowPulse = coolerSystem.lastFlowPulseMicros();
        if (lastFlowPulse != lastFlowPulseDisplayed) {
            if (pulseDivider++ % 2) {
                display.setLED(ScreenLED::GREEN, !display.getLED(ScreenLED::GREEN));
            }
            lastFlowPulseDisplayed = lastFlowPulse;
        }
    } else {
        display.setLED(ScreenLED::GREEN, false);
    }

    // blink error codes to red LED and external status LED
    bool statusLEDState = getSystemStatusLEDState();
    if (display.getLED(ScreenLED::RED) != statusLEDState) {
        systemStatusLED.setPercent(statusLEDState ? ClockTime::getDimmerBrightnessPercent() * SYSTEM_STATUS_LED_BASE_BRIGHTNESS / 100 : 0);
        display.setLED(ScreenLED::RED, statusLEDState);
    }

    if (!displayUpdate.check()) {
        return;
    }

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
        
    if (page == 9) {
        display.displayError(rtData.fault);
        return;
    }

    // output buffer size is 5 screen size is 3, why?
    // decimal point chars are collapsed to prior char
    // so they don't count and c_strs are null terminated
    char buf[5];
    switch (page) {
        case 0:
            dto3a(buf, rtData.evaporatorInletTemp);
            break;
        case 1:
            dto3a(buf, rtData.evaporatorOutletTemp);
            break;
        case 2:
            dto3a(buf, rtData.condenserInletTemp);
            break;
        case 3:
            dto3a(buf, rtData.condenserOutletTemp);
            break;
        case 4:
            uito3a(buf, rtData.compressorSpeed * 100);
            break;
        case 5:
            dto3a(buf, rtData.compressorCurrent);
            break;
        case 6:
            if (rtData.compressorFrequency) {
                dto3a(buf, rtData.compressorFrequency);
            } else {
                strcpy(buf, "---");
            }
            break;
        case 7:
            dto3a(buf, rtData.flowRate);
            break;
        case 8:
            uito3a(buf, rtData.systemPressure);
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
                faultBlinks = CoolerSystem::getFaultBlinks(rtData.fault, rtData.compressorFaultCode);
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
