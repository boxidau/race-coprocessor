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
    unsigned long lastFlowPulseDisplayed { 0 };
    uint8_t pulseDivider { 0 };

public:
    CoolerUI(
        CoolerSystem &_coolerSystem,
        uint _displayPin,
        uint _uiButtonPin
    )
        : coolerSystem { _coolerSystem }
        , display { Display(_displayPin) }
        , uiButtonPin { _uiButtonPin }
        , uiButton { Bounce(_uiButtonPin, 20) }
    {
    };
    void setup();

    void loop();
    bool shouldDisplayName();
    const CoolerSystemData& getCSData();
};