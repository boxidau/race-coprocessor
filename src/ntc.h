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
        const uint32_t _pullupResistance,
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
        float ntcResistanceApprox = pullupResistance / (((float) ADC_MAX / sample) - 1);
        // double fullScaleLeakageOffset = 1 / (1 / ntcResistanceApprox + 1 / pullupResistance) * FULL_SCALE_LEAKAGE_CURRENT / FULL_SCALE_VOLTAGE;
        // double offsetAdjustedVal = val + fullScaleLeakageOffset * val;
        // double ntcResistance = pullupResistance / ((FULL_SCALE_VREF / offsetAdjustedVal) - 1);
        float lnR = log(ntcResistanceApprox);
        // expand out lnR * lnR * lnR, much faster than pow()
        return 1 / (steinhartA + (steinhartB * lnR) + (steinhartC * lnR * lnR * lnR)) - 273.15;
    }

private:
    const uint32_t pullupResistance;
    const float steinhartA, steinhartB, steinhartC;
};

typedef BaseNTC<NTC_DEFAULT_SAMPLES> NTC;
typedef BaseNTC<NTC_PRECISION_SAMPLES> PrecisionNTC;
