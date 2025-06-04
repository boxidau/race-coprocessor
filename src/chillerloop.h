#include <Arduino.h>
#include <PID_v1.h>
#include "mafilter.h"
#include "constants.h"

#define COMPRESSOR_PID_KP 0.4 // 0.4/0.003 gives 100s time constant (90% -> 75%). 0.2/0.001 is slower, 150s.
#define COMPRESSOR_PID_KI 0.003
#define COMPRESSOR_PID_KD 0

// values interpreted from Buttonwillow 9/24 data
#define COMPRESSOR_STARTUP_DELAY_MS 3000 // time between pump and compressor start, to 1) allow temp to stabilize before deciding whether to turn on the compressor and 2) to allow driver switch selection to stabilize
// todo: try 0 here because previously we were initializing to 0.84, see what happens at 0.50
#define COMPRESSOR_PID_START_DELAY_MS 6000 // compressor on time before starting PID control, to allow temp filter to stabilize
//#define COMPRESSOR_PID_START_DELAY_MS 20000 // compressor on time before starting PID control, to allow temperature velocity to stabilize
#define COMPRESSOR_COOLDOWN_MS 60000 // durations w/o switchoff are 40s @ high, 60s @ med, 75s @ low; heating is ~100W compressor/100W driver; target 1.5x this and use high as worst case
#define COMPRESSOR_SPEED_UPDATE_MS 5000 // min time between speed changes to prevent chatter
#define CHILLER_PUMP_REMAIN_ON_MS 5000 // time the chiller pump remains on after compressor shutdown
#define CHILLER_PUMP_EARLY_SHUTDOWN_MODULUS 3 // period of A/B testing for chiller pump off vs on. first cycle is on, remainder are off
#define TEMP_MA_FILTER_SAMPLES 50 // 5s @ 0.1s per sample
#define TEMP_MA_FILTER_PRECISION 100000 // 1e-5 C

#define EVAPORATOR_OUTLET_CUTOFF_TEMP -1.0 // 10% IPA / 90% water freezes at -4C

#define FLOW_RATE_TARGET 3.4 // Lpm
#define CHILLER_PUMP_PID_KP (0.05 * 12) // multiply by 12 since gains were tested at that value
#define CHILLER_PUMP_PID_KI (0.25 * 12) // 0.5 also stable but has overshoot
#define CHILLER_PUMP_PID_KD 0
#define CHILLER_PUMP_MIN_SPEED 4.5 // Volts. pump is specced down to 5V and drops out at 4V
#define CHILLER_PUMP_MAX_SPEED 9 // Volts
#define CHILLER_PUMP_DEFAULT_SPEED 6 // Volts, when not using PID control

// speed steps occur at voltage ratios 0.470 (zero below this) .. 0.955.
// these translate to 0.50 .. 1.00 normalized.
// use midpoints of speed steps for setting compressor voltage.
#define NUM_COMPRESSOR_SPEEDS 7
#define LOWEST_OPERATING_COMPRESSOR_SPEED_INDEX 1 // 0.58
#define HIGHEST_OPERATING_COMPRESSOR_SPEED_INDEX 3 // 0.75
#define COMPRESSOR_SPEED_RATIO_TO_ANALOG (9 / (3.3 * 3.717) * ADC_MAX * 0.97)

const float CompressorSpeeds[NUM_COMPRESSOR_SPEEDS] = {
    0.50,
    0.58,
    0.67,
    0.75,
    0.83,
    0.92,
    1.00
};

// midpoint voltages
const float CompressorSpeedVoltageRatios[NUM_COMPRESSOR_SPEEDS] = {
    0.510, // transition point 0.470 (4.16V)
    0.591, // transition point 0.551
    0.672, // transition point 0.632
    0.753, // transition point 0.713
    0.834, // transition point 0.793
    0.915, // transition point 0.874
    0.995  // transition point 0.955 (8.45V)
};

enum class ChillerLoopState {
    OFF                    = 0,
    PUMP_ONLY              = 1,
    COMPRESSOR_AND_PUMP_ON = 2,
    COMPRESSOR_COOLDOWN    = 3
};

constexpr const char* chillerLoopStateToString(ChillerLoopState cls)
{
    switch (cls) {
        case ChillerLoopState::OFF: return "OFF";
        case ChillerLoopState::PUMP_ONLY: return "PUMP_ONLY";
        case ChillerLoopState::COMPRESSOR_AND_PUMP_ON: return "COMPRESSOR_AND_PUMP_ON";
        case ChillerLoopState::COMPRESSOR_COOLDOWN: return "COMPRESSOR_COOLDOWN";
        default: return "UNKNOWN";
    }
}

class ChillerLoopController {
    public:
        void setup(uint32_t sampleTime);
        void updateState(bool systemEnableRequested, float evapInletTemp, float restartTemp, float cutoffTemp, float evapOutletTemp, float flowRateInput);

        float getEvapInletTempFiltered() {
            return evapInletTempFiltered;
        }

        float getCompressorSpeedContinuous() {
            return compressorSpeed;
        }

        float getCompressorSpeedQuantized() {
            return compressorSpeed ? CompressorSpeeds[compressorSpeedIndex] : 0;
        }

        float getCompressorSpeedVoltage() {
            return compressorSpeed ? CompressorSpeedVoltageRatios[compressorSpeedIndex] * COMPRESSOR_SPEED_RATIO_TO_ANALOG : 0;
        }

        uint32_t getCompressorCooldownSecondsRemaining() {
            if (state != ChillerLoopState::COMPRESSOR_COOLDOWN) {
                return 0;
            }

            int32_t remaining = ceilf((float) (compressorShutdownTime + COMPRESSOR_COOLDOWN_MS - millis()) / 1000);
            return max(remaining, 0);
        }

        uint32_t getMillisSincePumpStart() {
            return millis() - pumpStartTime;
        }

        float getPumpSpeed() {
            return pumpSpeed;
        }

        ChillerLoopState getState() {
            return state;
        }

        const char* getStateString() {
            return chillerLoopStateToString(state);
        }

    private:
        void startCompressor();
        void shutdownCompressor();
        void enableCompressorPID();
        void updateCompressorSpeed();
        void startPump();
        void shutdownPump(); 
        void updatePumpSpeed();

        static uint32_t getQuantizedSpeedIndexFor(float compressorSpeed);

        ChillerLoopState state { ChillerLoopState::OFF };
        uint32_t time { 0 };
        uint32_t pumpStartTime { 0 };
        uint32_t compressorStartTime { 0 };
        uint32_t compressorShutdownTime { 0 };
        uint32_t compressorCycle { 0 };
        uint32_t lastCompressorSpeedChangeTime { 0 };

        float flowRate { 0 };
        float pumpSpeed { 0 };
        float flowRateTarget { FLOW_RATE_TARGET };
        PID chillerPumpPID {
            &flowRate,
            &pumpSpeed,
            &flowRateTarget,
            CHILLER_PUMP_PID_KP,
            CHILLER_PUMP_PID_KI,
            CHILLER_PUMP_PID_KD,
            P_ON_E,
            DIRECT
        };

        MAFilter<float, int32_t, TEMP_MA_FILTER_SAMPLES, TEMP_MA_FILTER_PRECISION> evapInletTempFilter;
        float evapInletTempFiltered { -100 };
        float evapInletTempTarget { 10 };
        float compressorSpeed { 0 };
        uint32_t compressorSpeedIndex { 0 };
        PID compressorPID {
            &evapInletTempFiltered,
            &compressorSpeed,
            &evapInletTempTarget,
            COMPRESSOR_PID_KP,
            COMPRESSOR_PID_KI,
            COMPRESSOR_PID_KD,
            P_ON_M,
            REVERSE
        };
};
