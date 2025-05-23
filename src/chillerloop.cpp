#include "chillerloop.h"

void ChillerLoopController::setup(uint32_t sampleTime) {
    compressorPID.SetOutputLimits(CompressorSpeeds[LOWEST_OPERATING_COMPRESSOR_SPEED_INDEX], CompressorSpeeds[HIGHEST_OPERATING_COMPRESSOR_SPEED_INDEX]);
    compressorPID.SetSampleTime(sampleTime);
    compressorPID.SetMode(MANUAL);

    chillerPumpPID.SetOutputLimits(CHILLER_PUMP_MIN_SPEED, CHILLER_PUMP_MAX_SPEED);
    chillerPumpPID.SetSampleTime(sampleTime);
    chillerPumpPID.SetMode(MANUAL);
}

void ChillerLoopController::updateState(bool systemEnableRequested, float evapInletTemp, float restartTemp, float cutoffTemp, float evapOutletTemp, float flowRateInput) {
    time = millis();
    evapInletTempFilter.push(evapInletTemp);
    evapInletTempFiltered = evapInletTempFilter.filteredValue();
    evapInletTempTarget = (cutoffTemp + restartTemp) / 2;
    flowRate = flowRateInput;

    switch (state) {
        case ChillerLoopState::OFF:
            if (systemEnableRequested) {
                // start only the pump, so we can measure temperature before we decide
                // whether to turn on the compressor
                state = ChillerLoopState::PUMP_ONLY;
                startPump();
                updatePumpSpeed();
            }
            break;

        case ChillerLoopState::PUMP_ONLY:
            if (!systemEnableRequested) {
                state = ChillerLoopState::OFF;
                shutdownPump();
                break;
            }

            updatePumpSpeed();
            // wait for the temperature to stabilize after starting the pump, and to prevent the compressor turning on/off immediately
            // and entering cooldown state if the driver clicks through off->prechill->low switch states
            if (time >= pumpStartTime + COMPRESSOR_STARTUP_DELAY_MS && evapInletTemp >= restartTemp) {
                state = ChillerLoopState::COMPRESSOR_AND_PUMP_ON;
                // todo: fixed prechill speed?
                startCompressor();
            }
            break;

        case ChillerLoopState::COMPRESSOR_AND_PUMP_ON:
            updatePumpSpeed();
            if (!systemEnableRequested || evapInletTemp < cutoffTemp || evapOutletTemp < EVAPORATOR_OUTLET_CUTOFF_TEMP) {
                // lower bound of evap inlet or outlet temp reached
                state = ChillerLoopState::COMPRESSOR_COOLDOWN;
                shutdownCompressor();
                break;
            }
            
            // allow time for temp velocity to stabilize to a lower value before starting PID control,
            // to avoid cranking up the speed only to reduce it rapidly
            if (time >= compressorStartTime + COMPRESSOR_PID_START_DELAY_MS) {
                enableCompressorPID();
            }

            updateCompressorSpeed();
            break;

        case ChillerLoopState::COMPRESSOR_COOLDOWN:
            // A/B test early pump shutdown (1 cycle no shutdown, N - 1 cycles early shutdown)
            if (time >= compressorShutdownTime + CHILLER_PUMP_REMAIN_ON_MS && pumpSpeed != 0 && compressorCycle % CHILLER_PUMP_EARLY_SHUTDOWN_MODULUS) {
                // shut down pump to reduce heat soak back into the evaporator
                shutdownPump();
            }

            // don't allow transitions out of this state until cooldown period ends, to reduce
            // startup stress on the compressor from ingesting liquid refrigerant, and to reduce
            // heat soak into the coolant
            if (time < compressorShutdownTime + COMPRESSOR_COOLDOWN_MS) {
                break;
            }

            if (systemEnableRequested) {
                state = ChillerLoopState::PUMP_ONLY;
                startPump();
            } else {
                state = ChillerLoopState::OFF;
            }
            break;
    }
}

void ChillerLoopController::startCompressor() {
    compressorStartTime = time;
    lastCompressorSpeedChangeTime = time;
    compressorSpeed = CompressorSpeeds[LOWEST_OPERATING_COMPRESSOR_SPEED_INDEX];
    compressorSpeedQuantized = compressorSpeed;
}

void ChillerLoopController::shutdownCompressor() {
    compressorShutdownTime = time;
    compressorPID.SetMode(MANUAL);
    compressorSpeed = 0;
    compressorSpeedQuantized = 0;
    compressorCycle++;
}

void ChillerLoopController::enableCompressorPID() {
#if USE_COMPRESSOR_PID
    compressorPID.SetMode(AUTOMATIC);
#endif
}

void ChillerLoopController::updateCompressorSpeed() {
    compressorPID.Compute();
    float newCompressorSpeedQuantized = getQuantizedSpeedFor(compressorSpeed);
    // set a minimum time in between speed changes to prevent chatter and reduce impact of temp noise
    if (compressorSpeedQuantized != newCompressorSpeedQuantized && time >= lastCompressorSpeedChangeTime + COMPRESSOR_SPEED_UPDATE_MS) {
        compressorSpeedQuantized = newCompressorSpeedQuantized;
        lastCompressorSpeedChangeTime = time;
    }
}

void ChillerLoopController::startPump() {
    pumpStartTime = time;
    pumpSpeed = CHILLER_PUMP_DEFAULT_SPEED;
#if USE_CHILLER_PUMP_PID
    chillerPumpPID.SetMode(AUTOMATIC);
#endif
}

void ChillerLoopController::shutdownPump() {
    chillerPumpPID.SetMode(MANUAL);
    pumpSpeed = 0;
}

void ChillerLoopController::updatePumpSpeed() {
    chillerPumpPID.Compute();
}

float ChillerLoopController::getQuantizedSpeedFor(float compressorSpeed) {
    // map continuous speed to quantized value, rounding inbetween states
    uint32_t index = round(constrain(map(compressorSpeed, CompressorSpeeds[0], CompressorSpeeds[NUM_COMPRESSOR_SPEEDS - 1], 0, NUM_COMPRESSOR_SPEEDS - 1), 0, NUM_COMPRESSOR_SPEEDS - 1));
    return CompressorSpeeds[index];
}
