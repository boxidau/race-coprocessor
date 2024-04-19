#include <math.h>
#include "baseadc.h"

#define NTC_DEFAULT_SAMPLES 100
#define NTC_PRECISION_SAMPLES 100

#define FULL_SCALE_LEAKAGE_CURRENT 0.0307e-6 // Amps @ FS (65535), assume this scales linearly with input voltage
#define FULL_SCALE_VOLTAGE 3.3 // Volts
#define FULL_SCALE_VREF 65516 // FS reading @ 3.3V @ zero leakage

template <int NTC_SAMPLES> class BaseNTC : public BaseADC<NTC_SAMPLES>
{
public:
    BaseNTC(
        const uint8_t _pin,
        const uint8_t _adcNum,
        const float _pullupResistance,
        const float _steinhartA,
        const float _steinhartB,
        const float _steinhartC
    )
        : BaseADC<NTC_SAMPLES>(_pin, _adcNum)
        , pullupResistance { _pullupResistance }
        , steinhartA { _steinhartA }
        , steinhartB { _steinhartB }
        , steinhartC { _steinhartC }
    {
    };

    float temperature() {
        return temperatureFor(this->adc());
    }

    float temperatureFor(uint16_t sample) {
        float ntcResistance = pullupResistance * sample / (ADC_MAX - sample);
        float lnR = logf(ntcResistance);
        // expand out lnR * lnR * lnR, much faster than pow()
        return 1 / (steinhartA + (steinhartB * lnR) + (steinhartC * lnR * lnR * lnR)) - 273.15;
    }

private:
    const float pullupResistance, steinhartA, steinhartB, steinhartC;
};

typedef BaseNTC<NTC_DEFAULT_SAMPLES> NTC;
typedef BaseNTC<NTC_PRECISION_SAMPLES> PrecisionNTC;
