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
#define LDR_THRESHOLD 36000
#define LDR_DEAD_TIME_RESET_MS 600
#define LDR_DEBOUNCE_MS 50

class CompressorFault
{
private:
    uint8_t pin;
    ABounce bounce;
    bool recordingFault = { false };
    uint8_t edges = { 0 };
    CompressorFaultCode code { CompressorFaultCode::OK };

public:
    CompressorFault(const uint8_t _pin) : pin { _pin }, bounce { ABounce(_pin, LDR_DEBOUNCE_MS, LDR_THRESHOLD) } {
    };
    
    void setup() {
        pinMode(pin, INPUT);

    }

    void loop() {
        bounce.update();
        if (!(millis() % 100)) LOG_INFO("100ms value ", analogRead(pin));
        if (!recordingFault && bounce.fallingEdge()) {
            LOG_INFO("first falling edge, ", millis(), analogRead(pin));
            recordingFault = true;
            edges = 1;
        } else if (recordingFault && bounce.fallingEdge()) {
            LOG_INFO("falling edge ", edges, ", ", millis(), analogRead(pin));
            edges++;
        } else if (recordingFault && bounce.duration() > LDR_DEAD_TIME_RESET_MS) {
            LOG_INFO("dead time reset, ", millis());
            if (edges == 0 || (edges >= 2 && edges <= 9)) {
                code = (CompressorFaultCode) edges;
            } else {
                LOG_ERROR("Invalid compressor fault code ", (uint8_t)code, ", returning UNKNOWN");
                code = CompressorFaultCode::UNKNOWN;
            }

            edges = 0;
            recordingFault = false;
        }
    }

    CompressorFaultCode getCode() {
        return code;
    }

    unsigned int durationSinceFaultRecorded() {
        if (code == CompressorFaultCode::OK) {
            return 0;
        }

        return bounce.duration();
    }
};