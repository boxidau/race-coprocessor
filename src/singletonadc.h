#pragma once

#include <ADC.h>

class SingletonADC {
    public:
        static ADC* getADC();
};
