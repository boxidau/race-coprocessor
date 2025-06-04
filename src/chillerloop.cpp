#include "chillerloop.h"

void ChillerLoopController::setup(uint32_t sampleTime) {
    compressorPID.SetOutputLimits(CompressorSpeeds[LOWEST_OPERATING_COMPRESSOR_SPEED_INDEX], CompressorSpeeds[HIGHEST_OPERATING_COMPRESSOR_SPEED_INDEX]);
    compressorPID.SetSampleTime(sampleTime);
    compressorPID.SetMode(MANUAL);

    chillerPumpPID.SetOutputLimits(CHILLER_PUMP_MIN_SPEED, CHILLER_PUMP_MAX_SPEED);
    chillerPumpPID.SetSampleTime(sampleTime);
    chillerPumpPID.SetMode(MANUAL);
}

void ChillerLoopController::updateState(bool systemEnableRequested, float evapInletTemp, float restartTemp, float cutoffTemp, float evapOutletTemp, float flowRateInput, float compressorCurrentInput) {
    time = millis();
    evapInletTempFilter.push(evapInletTemp);
    evapInletTempFiltered = evapInletTempFilter.filteredValue();
    evapInletTempTarget = (cutoffTemp + restartTemp) / 2;
    flowRate = flowRateInput;
    compressorCurrent = compressorCurrentInput;

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
            // and entering cooldown state if the driver clicks through off->prechill->low switch states. it's ok to turn on compressor
            // immediately if clicking from med->high states, however, since it's unlikely the driver will click med->high->med->low
            // therefore debouncing is unnecessary.
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
            
            // allow time for temp filter to stabilize before starting PID control.
            // todo: consider whether we also need temp velocity to stabilize with compressor running
            if (time >= compressorStartTime + COMPRESSOR_PID_START_DELAY_MS) {
                enableCompressorPID();
            }

            updateCompressorSpeed();
            break;

        case ChillerLoopState::COMPRESSOR_COOLDOWN:
            updatePumpSpeed();

            // A/B test early pump shutdown (first N - 1 cycles early shutdown, 1 cycle no shutdown, repeat)
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
                if (pumpSpeed == 0) {
                    startPump();
                    updatePumpSpeed();
                }
            } else {
                state = ChillerLoopState::OFF;
                if (pumpSpeed != 0) {
                    shutdownPump();
                }
            }
            break;
    }
}

void ChillerLoopController::startCompressor() {
    compressorStartTime = time;
    lastCompressorSpeedChangeTime = time;
    compressorSpeed = CompressorSpeeds[LOWEST_OPERATING_COMPRESSOR_SPEED_INDEX];
    compressorSpeedIndex = LOWEST_OPERATING_COMPRESSOR_SPEED_INDEX;
}

void ChillerLoopController::shutdownCompressor() {
    compressorShutdownTime = time;
    compressorPID.SetMode(MANUAL);
    compressorSpeed = 0;
    compressorSpeedIndex = 0;
    compressorCycle++;
}

void ChillerLoopController::enableCompressorPID() {
#if USE_COMPRESSOR_PID
    compressorPID.SetMode(AUTOMATIC);
#endif
}

void ChillerLoopController::updateCompressorSpeed() {
    if (compressorCurrent >= COMPRESSOR_MAX_CURRENT_LIMIT && time >= lastCompressorSpeedChangeTime + COMPRESSOR_SPEED_UPDATE_MS) {
        // reduce the maximum speed of the compressor to limit current draw. this is necessary in high ambient/condenser temp
        // conditions where the compressor can draw 40A+ at 100% speed
        if (compressorMaxSpeedIndex > LOWEST_OPERATING_COMPRESSOR_SPEED_INDEX) {
            compressorMaxSpeedIndex--;
        }

        compressorPID.SetOutputLimits(CompressorSpeeds[LOWEST_OPERATING_COMPRESSOR_SPEED_INDEX], CompressorSpeeds[compressorMaxSpeedIndex]);
    }

    compressorPID.Compute();
    uint32_t newCompressorSpeedIndex = getQuantizedSpeedIndexFor(compressorSpeed);
    // set a minimum time in between speed changes to prevent chatter and reduce impact of temp noise
    if (compressorSpeedIndex != newCompressorSpeedIndex && time >= lastCompressorSpeedChangeTime + COMPRESSOR_SPEED_UPDATE_MS) {
        compressorSpeedIndex = newCompressorSpeedIndex;
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

uint32_t ChillerLoopController::getQuantizedSpeedIndexFor(float compressorSpeed) {
    // map continuous speed to quantized value, rounding inbetween states
    return round(constrain(map(compressorSpeed, CompressorSpeeds[0], CompressorSpeeds[NUM_COMPRESSOR_SPEEDS - 1], 0, NUM_COMPRESSOR_SPEEDS - 1), 0, NUM_COMPRESSOR_SPEEDS - 1));
}
