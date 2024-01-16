#include "Arduino.h"

#include <DebugLog.h>
#include <SPI.h>

#define DISPLAY_SIZE 3
#define SCREEN_BUF_LED_IDX 3

const SPISettings MAX6950CEESettings(200000, MSBFIRST, SPI_MODE0);
const byte SCREEN_CHAR0 = 0x60;
const byte SCREEN_CHAR1 = 0x61;
const byte SCREEN_CHAR2 = 0x62;
const byte SCREEN_LED   = 0x63;  // used for raw LEDs

enum class MAX6950Address {
    DECODE_MODE = 0x01,
    INTENSITY = 0x02,
    SCAN_LIMIT = 0x03,
    CONFIGURATION = 0x04,
};

enum class ScreenLED {
    RED    = B00010000,
    YELLOW = B00001000,
    GREEN  = B00000100
};


constexpr const byte CharToSegValue(char c_input)
{
    switch (c_input)
    {
        case '0': return B01111110;
        case '1': return B00110000;
        case '2': return B01101101;
        case '3': return B01111001;
        case '4': return B00110011;
        case '5': return B01011011;
        case '6': return B01011111;
        case '7': return B01110000;
        case '8': return B01111111;
        case '9': return B01111011;
        case 'A': return B01110111;
        case 'B': return B00011111;
        case 'C': return B01001110;
        case 'D': return B00111101;
        case 'E': return B01001111;
        case 'F': return B01000111;
        case 'G': return B01011110;
        case 'H': return B00110111;
        case 'I': return B00000100;
        case 'J': return B00111100;
        case 'L': return B00001110;
        case 'M': return B01010101;
        case 'N': return B00010101;
        case 'O': return B00011101;
        case 'P': return B01100111;
        case 'R': return B00000101;
        case 'T': return B00001111;
        case 'U': return B00011100;
        case 'W': return B01011100;
        case 'Y': return B00111011;
        case '-': return B00000001;
        default:  return B00000000;
    }
}

class Display {
private:
    const uint8_t chipSelectPin;
    byte screen[4] = { 0, 0, 0, 0 };
    void updateDisplay();
public:
    Display(
        const uint8_t _chipSelectPin
    )
        : chipSelectPin { _chipSelectPin }
    {};
    void setup();
    void setString(const char *str);
    void setLED(ScreenLED led, bool value);
    bool getLED(ScreenLED led);

    void displayError(uint8_t errorCode);
    String debugString();
};