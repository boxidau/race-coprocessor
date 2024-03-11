#include "display.h"

void sendSPIData(uint8_t csPin, uint8_t address, uint8_t value) {
    SPI1.beginTransaction(MAX6950CEESettings);
    digitalWrite(csPin, LOW);
    SPI1.transfer16(address << 8 | value);
    digitalWrite(csPin, HIGH);
    SPI1.endTransaction();
}

void Display::updateDisplay() {
    static uint32_t previousData = 0;
    uint32_t currentData = (uint32_t)screen[0] << 24 |
      (uint32_t)screen[1] << 16 |
      (uint32_t)screen[2] << 8  |
      (uint32_t)screen[3];

    if (currentData != previousData) {
        previousData = currentData;
        sendSPIData(chipSelectPin, SCREEN_CHAR0, screen[0]);
        sendSPIData(chipSelectPin, SCREEN_CHAR1, screen[1]);
        sendSPIData(chipSelectPin, SCREEN_CHAR2, screen[2]);
        sendSPIData(chipSelectPin, SCREEN_LED, screen[SCREEN_BUF_LED_IDX]);
    }
};

void Display::setup() {
    SPI1.begin();
    pinMode(chipSelectPin, OUTPUT);
    digitalWrite(chipSelectPin, HIGH);
    // intensity set to 15/16
    sendSPIData(chipSelectPin, 0x02, 0x0A);
    // disable test mode
    sendSPIData(chipSelectPin, 0x07, 0x00);
    // set initial configuration
    sendSPIData(chipSelectPin, 0x04, 0x01);

}

void Display::setString(const char *str) {
    int screenIdx = 2;
    bool dp = false;
    for (auto i = strlen(str)-1 ; i >= 0; i--) {
        if (str[i] == '.') {
            dp = true;
        } else {
            screen[screenIdx] = CharToSegValue(str[i]);
            if (dp) screen[screenIdx] |= 0x80;
            dp = false;
            screenIdx--;
        }
        if (screenIdx < 0) {
            break;
        }
    }
    // is the incoming string buffer is shorter than the screen
    // fill/reset the remaining chars
    while (screenIdx >= 0) {
        screen[screenIdx] = 0;
        screenIdx--;
    }
    updateDisplay();
};

void Display::setLED(ScreenLED led, bool value) {
    if (value) {
        screen[SCREEN_BUF_LED_IDX] |= (byte)led;
    } else {
        screen[SCREEN_BUF_LED_IDX] &= ~(byte)led;
    }
    updateDisplay();
};

bool Display::getLED(ScreenLED led) {
    return screen[SCREEN_BUF_LED_IDX] & (byte)led;
};

void Display::displayError(uint8_t errorCode) {
    errorCode = errorCode % 100;
    char buf[4];
    snprintf(buf, 4, "E%02d", errorCode);
    setString(buf);
}

String Display::debugString() {
    String line1 = "           \n";
    String line2 = "           \n";
    String line3 = "           \n";
    String line4 = " O   O   O \n";
    for (int i = 0; i < DISPLAY_SIZE; i++) {
        uint8_t digitOffset = i * 4;
        if (screen[i] & B10000000) {
            line3[digitOffset+3] = '.';
        }
        if (screen[i] & B01000000) {
            line1[digitOffset+1] = '_';
        }
        if (screen[i] & B00100000) {
            line2[digitOffset+2] = '|';
        }
        if (screen[i] & B00010000) {
            line3[digitOffset+2] = '|';
        }
        if (screen[i] & B00001000) {
            line3[digitOffset+1] = '_';
        }
        if (screen[i] & B00000100) {
            line3[digitOffset] = '|';
        }
        if (screen[i] & B00000010) {
            line2[digitOffset] = '|';
        }
        if (screen[i] & B00000001) {
            line2[digitOffset+1] = '_';
        }
    }
    if (getLED(ScreenLED::RED)) {
        line4[1] = 'R';
    }
    if (getLED(ScreenLED::YELLOW)) {
        line4[5] = 'Y';
    }
    if (getLED(ScreenLED::GREEN)) {
        line4[9] = 'G';
    }

    return line1 + line2 + line3 + line4;
}