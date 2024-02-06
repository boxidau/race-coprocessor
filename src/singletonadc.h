#pragma once

#include <arduino.h>
#include <ADC.h>

#include "constants.h"


class SingletonADC {
    public:
        static ADC* getADC();
};
