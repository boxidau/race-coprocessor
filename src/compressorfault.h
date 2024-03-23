#pragma once

#include "Arduino.h"
#include <ABounce.h>

enum class CompressorFaultCode {
    OK = 0,
    HIGH_CURRENT = 2,
    MOTOR_BLOCKED = 3,
    UNDER_VOLTAGE = 4,
    FAN_FAILURE = 6,
    COMPRESSOR_OFFLINE = 7,
    COMPRESSOR_OVERHEAT = 8,
    SYSTEM_OVERPRESSURE = 9,
    UNKNOWN = 10,
};

constexpr const char* CompressorFaultToString(CompressorFaultCode f)
{
    switch (f)
    {
        case CompressorFaultCode::OK: return "OK";
        case CompressorFaultCode::HIGH_CURRENT: return "HIGH_CURRENT";
        case CompressorFaultCode::MOTOR_BLOCKED: return "MOTOR_BLOCKED";
        case CompressorFaultCode::UNDER_VOLTAGE: return "UNDER_VOLTAGE";
        case CompressorFaultCode::FAN_FAILURE: return "FAN_FAILURE";
        case CompressorFaultCode::COMPRESSOR_OFFLINE: return "COMPRESSOR_OFFLINE";
        case CompressorFaultCode::COMPRESSOR_OVERHEAT: return "COMPRESSOR_OVERHEAT";
        case CompressorFaultCode::SYSTEM_OVERPRESSURE: return "SYSTEM_OVERPRESSURE";
        default: return "UNKNOWN";
    }
}

// Waveform is 400ms pulse period, ~2ms fall time / 10ms rise time, 1.5s reset gap between sequences.
// Resistance changes 1kohm - 90kohm, falling edge is much faster.
// Steady state (no fault) ADC value is ~63000. Waveform on/off range is 13000/56000.
#define LDR_THRESHOLD 35000
#define LDR_DEAD_TIME_RESET_MS 600
#define LDR_DEBOUNCE_MS 50

class CompressorFault
{
private:
    uint8_t pin;
    ABounce bounce;
    bool recordingFault = { false };
    uint8_t blinks = { 0 };
    CompressorFaultCode code { CompressorFaultCode::OK };

public:
    CompressorFault(const uint8_t _pin) : pin { _pin }, bounce { ABounce(_pin, LDR_DEBOUNCE_MS, LDR_THRESHOLD) } {
    };
    
    void setup() {
        pinMode(pin, INPUT_DISABLE);

    }

    void loop() {
        bounce.update();
        if (!recordingFault && bounce.fallingEdge()) {
            recordingFault = true;
            blinks = 1;
        } else if (recordingFault && bounce.fallingEdge()) {
            blinks++;
        } else if (recordingFault && bounce.duration() > LDR_DEAD_TIME_RESET_MS) {
            if (blinks >= 2 && blinks <= 9) {
                code = (CompressorFaultCode) blinks;
                LOG_INFO("Logged compressor fault", CompressorFaultToString(code));
            } else {
                LOG_ERROR("Invalid compressor fault,", blinks, "blinks, returning UNKNOWN");
                code = CompressorFaultCode::UNKNOWN;
            }

            blinks = 0;
            recordingFault = false;
        }
    }

    CompressorFaultCode getCode() {
        return code;
    }

    void reset() {
        code = CompressorFaultCode::OK;
    }

    uint32_t durationSinceFaultRecorded() {
        if (code == CompressorFaultCode::OK) {
            return 0;
        }

        return bounce.duration();
    }
};