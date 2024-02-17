#pragma once

#include "Arduino.h"
#include <TimeLib.h>

class LoopTimer {
    public:
        uint32_t start() {
            unsigned long time = micros();
            _prevLoop = _hasStarted ? _prevLoop : time;
            unsigned long duration = time >= _prevLoop ? time - _prevLoop : time + (UINT32_MAX - _prevLoop);
            _elapsedSinceStart += duration;
            _prevLoop = time;
            _hasStarted = true;
            return duration;
        };

        bool hasStarted() {
            return _hasStarted;
        };

        uint64_t elapsedSinceStart() {
            return _elapsedSinceStart;
        };

        uint32_t elapsedSinceLoop() {
            unsigned long time = micros();
            return time >= _prevLoop ? time - _prevLoop : time + (UINT32_MAX - _prevLoop);
        };

    private:
        unsigned long _prevLoop { 0 };
        uint64_t _elapsedSinceStart { 0 };
        bool _hasStarted { false };
};
