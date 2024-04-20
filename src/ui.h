#include "Arduino.h"
#include <Bounce.h>
#include <Metro.h>
#include "display.h"
#include "coolersystem.h"

class CoolerUI {
private:
    CoolerSystem &coolerSystem;
    Display display;
    Metro displayUpdate { Metro(100) };
    uint uiButtonPin;
    Bounce uiButton;
    uint page { 0 };
    uint displayPageNameUntil { 0 };
    CoolerSystemData rtData { CoolerSystemData() };
    bool pageTurnerEnabled { true };
    uint32_t lastFlowPulseDisplayed { 0 };
    uint8_t pulseDivider { 0 };

    uint8_t systemStatusLEDPin;
    uint32_t faultBlinks { 0 };
    uint32_t currentBlink { 0 };
    uint32_t lastBlinkTime { 0 };

    bool shouldDisplayName();
    bool getSystemStatusLEDState();
    uint32_t getFaultBlinks();

public:
    CoolerUI(
        CoolerSystem &_coolerSystem,
        uint _displayPin,
        uint _uiButtonPin,
        uint _systemStatusLEDPin
    )
        : coolerSystem { _coolerSystem }
        , display { Display(_displayPin) }
        , uiButtonPin { _uiButtonPin }
        , uiButton { Bounce(_uiButtonPin, 20) }
        , systemStatusLEDPin(_systemStatusLEDPin)
    {
    };

    void setup();
    void loop();
};