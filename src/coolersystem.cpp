#include "coolersystem.h"
#include "clocktime.h"
#include "datasdlogger.h"

#if ANF_SAMPLES_TEST
#include "../test/compressorcurrentsamples.h"
#endif

void printFaultLine(StringFormatDisplay& format, SystemFault f, byte systemFault) {
    const char* faultName = SystemFaultToString(f);
    size_t faultLen = strlen(faultName);
    size_t prefLength = faultLen + ((uint32_t) f > 10 ? 2 : 1);
    format.formatLiteral("  ");
    format.formatString(faultName, faultLen);
    format.formatLiteral(" ( ");
    format.formatUnsignedInt((uint32_t) f);
    format.formatString(" ):             ", max(28 - (int) prefLength, 0));
    systemFault & (byte)f ? format.formatLiteral("ALARM\n") : format.formatLiteral("OK\n");
}

void CoolerSystem::setupIO()
{
    // setup pin modes and initial states
    systemEnableOutput.setup();
    coolshirtPWM.setup();
    chillerPumpPWM.setup();

    flowSensor.setup();
    pinMode(coolantLevelPin, INPUT);
    compressorFault.setup();

    evaporatorInletNTC.setup();
    evaporatorOutletNTC.setup();
    condenserInletNTC.setup();
    condenserOutletNTC.setup();
    ambientNTC.setup();

    switchADC.setup();
    pressureSensor.setup();
    pressureSensor.setCalibration(
        PRESSURE_SENSOR_CALIBRATION_LOW_ADC,
        PRESSURE_SENSOR_CALIBRATION_LOW_KPA,
        PRESSURE_SENSOR_CALIBRATION_HIGH_ADC,
        PRESSURE_SENSOR_CALIBRATION_HIGH_KPA,
        true
    );

    currentSensor.setup();
    currentSensor.setCalibration(
        CURRENT_SENSOR_CALIBRATION_LOW_ADC,
        CURRENT_SENSOR_CALIBRATION_LOW_AMPS,
        CURRENT_SENSOR_CALIBRATION_HIGH_ADC,
        CURRENT_SENSOR_CALIBRATION_HIGH_AMPS,
        true
    );

    pinMode(compressorSpeedPin, OUTPUT);
    analogWrite(compressorSpeedPin, 0);
    compressorPID.SetOutputLimits(COMPRESSOR_MIN_SPEED_RATIO, COMPRESSOR_MAX_SPEED_RATIO);
    compressorPID.SetSampleTime(UPDATE_STATE_TIMER_MS);
    compressorPID.SetMode(MANUAL);

    chillerPumpPID.SetOutputLimits(CHILLER_PUMP_MIN_SPEED, CHILLER_PUMP_MAX_SPEED);
    chillerPumpPID.SetSampleTime(UPDATE_STATE_TIMER_MS);
    chillerPumpPID.SetMode(MANUAL);

    voltageMonitor.setup();

#if ANF_SAMPLES_TEST
    analyzeNoteFrequency.begin();
#endif
}

void CoolerSystem::setupLogging()
{
#if NTC_DEBUG
    //sampleLogger.ensureSetup("time,current,biquad,12V");
    sampleLogger.ensureSetup("time,inlet,outlet");
#elif FLOW_DEBUG
    sampleLogger.ensureSetup("time,index,duration,chillerPump,coolantLevel");
#endif

    DataSDLogger::setup();
    DataSDLogger::logData(getLogHeader());
}

void CoolerSystem::pollCoolantLevel()
{
    if (coolantLevelBounce.update()) {
        coolantLevel = coolantLevelBounce.read();
    }
}

void CoolerSystem::runChillerPump()
{
    // run chiller pump whenever the compressor is on
    if (systemEnableOutput.value()) {
        if (!chillerPumpPWM.value()) {
            pumpStartTime = millis();
            chillerPumpSpeed = CHILLER_PUMP_DEFAULT_SPEED;
#if USE_CHILLER_PUMP_PID
            // initialize speed to 0 so PID controller doesn't start bumpless control starting at the default speed.
            // consider initializing to max for faster response?
            chillerPumpSpeed = 0;
            chillerPumpPID.SetMode(AUTOMATIC);
#endif
#if FLOW_DEBUG
            // log the exact time the pump started running
            sampleLogger.logSamples(ClockTime::millisSinceEpoch(), 0, 0, 1, 0);
#endif
        }

        chillerPumpPID.Compute();
        // scale PWM output by the system voltage, with safety net in case it glitches low
        chillerPumpPWM.set(roundf(chillerPumpSpeed / max(voltageMonitor.get12vMilliVolts(), 10000) * 1000 * ADC_MAX));
        //LOG_INFO(ClockTime::secSinceEpoch(), chillerPumpSpeed, instantaneousFlowRate, flowRate);
        return;
    }

    // compressor is off
    if (chillerPumpPWM.value()) {
        // shut down pump
#if FLOW_DEBUG
        sampleLogger.logSamples(ClockTime::millisSinceEpoch(), 0, 0, 0, 0);
#endif

        chillerPumpPID.SetMode(MANUAL);
        chillerPumpPWM.set(0);
        chillerPumpSpeed = 0;
        return;
    }
}

void CoolerSystem::runCompressor()
{
    // calculate temperature thresholds
    float cutoffTemp = COMPRESSOR_UNDER_TEMP_CUTOFF_HIGH, restartTemp = COMPRESSOR_RESTART_TEMP_HIGH;
    switch (systemStatus) {
        case CoolerSystemStatus::PUMP_LOW:
            cutoffTemp = COMPRESSOR_UNDER_TEMP_CUTOFF_LOW;
            restartTemp = COMPRESSOR_RESTART_TEMP_LOW;
            break;
        case CoolerSystemStatus::PUMP_MEDIUM:
        case CoolerSystemStatus::PRECHILL:
            cutoffTemp = COMPRESSOR_UNDER_TEMP_CUTOFF_MED;
            restartTemp = COMPRESSOR_RESTART_TEMP_MED;
            break;            
        case CoolerSystemStatus::PUMP_HIGH:
            cutoffTemp = COMPRESSOR_UNDER_TEMP_CUTOFF_HIGH;
            restartTemp = COMPRESSOR_RESTART_TEMP_HIGH;
            break;
        default:
            shutdownCompressor();
            return;
    }
    compressorTempTarget = (cutoffTemp + restartTemp) / 2;

    // implement hysteresis on the undertemp cutoff. this isn't an error condition
    // but we are going to shut down the compressor until temp goes back above a safe value
    if (evaporatorInletTemp < cutoffTemp) {
        undertempCutoff = true;
    }
    if (evaporatorInletTemp >= restartTemp) {
        undertempCutoff = false;
    }

    if (undertempCutoff) {
        shutdownCompressor();
        return;
    }
    
    startupCompressor();

    if (evaporatorInletTemp >= restartTemp + 2.0 && compressorSpeed < COMPRESSOR_DEFAULT_SPEED) {
        // current speed isn't cutting it, kick to something higher
        compressorSpeed = COMPRESSOR_DEFAULT_SPEED;
    }

    compressorPID.Compute();
    //LOG_INFO(ClockTime::secSinceEpoch(), compressorSpeed, evaporatorInletTemp);
    analogWrite(compressorSpeedPin, roundf(compressorSpeed * COMPRESSOR_SPEED_RATIO_TO_ANALOG));
}

void CoolerSystem::shutdownCompressor()
{
    if (systemEnableOutput.value()) {
        compressorShutoffTime = millis();
        compressorSpeed = 0;
        systemEnableOutput.setBoolean(false);
        analogWrite(compressorSpeedPin, 0);
        compressorPID.SetMode(MANUAL);

        analyzeNoteFrequency.stop();
        compressorFrequency = 0;
        compressorFrequencyProbability = 0;
    }
}

void CoolerSystem::startupCompressor()
{
    if (!systemEnableOutput.value() &&
        (!compressorShutoffTime || millis() >= compressorShutoffTime + COMPRESSOR_MIN_COOLDOWN_MS)) {
        systemEnableOutput.setBoolean(true);

        // rotate through compressor speeds to gather data on each one,
        // unless we're in prechill
        switch(systemStatus) {
            case CoolerSystemStatus::PUMP_LOW:
            case CoolerSystemStatus::PUMP_MEDIUM:
            case CoolerSystemStatus::PUMP_HIGH:
                compressorSpeed = CompressorSpeeds[compressorSpeedIndex];
                compressorSpeedIndex++;
                if (compressorSpeedIndex == sizeof(CompressorSpeeds)) {
                    compressorSpeedIndex = 0;
                }
                break;

            default:
                compressorSpeed = COMPRESSOR_DEFAULT_SPEED;
                break;
        }

#if USE_COMPRESSOR_PID
        // initialize speed to 0 so PID controller doesn't start bumpless control starting at the default speed
        compressorSpeed = 0;
        compressorPID.SetMode(AUTOMATIC);
#endif

        // reset note frequency analyzer to start with fresh data
        analyzeNoteFrequency.begin();
    }
}

void CoolerSystem::runCoolshirtPump()
{
    switch(systemStatus) {
        case CoolerSystemStatus::PUMP_LOW:
        case CoolerSystemStatus::PUMP_MEDIUM:
        case CoolerSystemStatus::PUMP_HIGH:
        case CoolerSystemStatus::FLUSH:
            coolshirtPWM.setPercent(100);
            return;

        default:
            coolshirtPWM.setPercent(0);
            return;
    }
}

void CoolerSystem::check(bool checkResult, SystemFault fault)
{
    if (!checkResult) {
        // fault code raised, assertion is false
        _systemFault |= (byte)fault;
    }
}

void CoolerSystem::acquireSamples()
{
    evaporatorInletNTC.loop();
    evaporatorOutletNTC.loop();
    condenserInletNTC.loop();
    condenserOutletNTC.loop();
    ambientNTC.loop();

    pressureSensor.loop();
    currentSensor.loop();
    compressorFault.loop();
    switchADC.loop();
    voltageMonitor.loop();

    pollCoolantLevel();

#if ANF_SAMPLES_TEST
    // float testWaveform = 
    //     (sampleCounter < 1000 ? sinf(2 * PI * 60 / 1000 * sampleCounter) : 0) + 
    //     (sampleCounter < 1000 ? 0.3 * sinf(2 * PI * 70 / 1000 * sampleCounter) : 0) + 
    //     (sampleCounter >= 1000 && sampleCounter < 2000 ? sinf(2 * PI * 40 / 1000 * sampleCounter) : 0) +
    //     (sampleCounter >= 2000 && sampleCounter < 3000 ? sinf(2 * PI * 80 / 1000 * sampleCounter) : 0) +
    //     4.0 * (sampleCounter > 1500) +
    //     0;

    // uint16_t currentSample = sampleCounter < 20000 ? compressorCurrentFullRun1[sampleCounter] : 0;
    uint16_t currentSample =
        sampleCounter < 3000 ? compressorCurrent50Percent[sampleCounter] : 0 +
        sampleCounter >= 3000 && sampleCounter < 6000 ? compressorCurrent60Percent[sampleCounter - 3000] : 0 +
        sampleCounter >= 6000 && sampleCounter < 9000 ? compressorCurrent70Percent[sampleCounter - 6000] : 0 +
        sampleCounter >= 9000 && sampleCounter < 12000 ? compressorCurrent80Percent[sampleCounter - 9000] : 0 +
        sampleCounter >= 12000 && sampleCounter < 15000 ? compressorCurrent90Percent[sampleCounter - 12000] : 0 +
        sampleCounter >= 15000 && sampleCounter < 18000 ? compressorCurrent100Percent[sampleCounter - 15000] : 0;

    compressorCurrentBiquadOutput = biquad.process(currentSample) * 20;

    // static uint32_t idx = 0;
    // static float arr[1000];
    // arr[idx++] = compressorCurrentBiquadOutput;
    // if (idx == 1000) {
    //     uint64_t rms = 0;
    //     uint32_t max = 0;
    //     for (int i = 0; i < 1000; i++) {
    //         max = ::max(arr[i], max);
    //         rms += arr[i] * arr[i];
    //     }
    //     idx = 0;
    //     LOG_INFO("biquad max: t=", ClockTime::secSinceEpoch(), "s, max", (uint16_t) max, "rms", sqrtf(rms / 1000));
    // }

    analyzeNoteFrequency.update(compressorCurrentBiquadOutput);

    if (analyzeNoteFrequency.available()) {
        LOG_INFO("analyzenotefreq: t=", sampleCounter, ClockTime::secSinceEpoch(), "s, valid result", analyzeNoteFrequency.validResult(), ", freq ", analyzeNoteFrequency.read(), "Hz, probability", analyzeNoteFrequency.probability());
    }
#else
    compressorCurrentBiquadOutput = biquad.process(currentSensor.latest()) * 20;
    analyzeNoteFrequency.update(compressorCurrentBiquadOutput);
#endif

    sampleCounter++;
}

void CoolerSystem::updateCoolerData() {
    flowRate = flowSensor.flowRate();
    instantaneousFlowRate = flowSensor.instantaneousFlowRate();
    systemPressure = pressureSensor.calibratedValue();
    compressorCurrent = (float) currentSensor.calibratedValue() / 1000; // mA -> A
    evaporatorInletTemp = evaporatorInletNTC.temperature();
    evaporatorOutletTemp = evaporatorOutletNTC.temperature();
    condenserInletTemp = condenserInletNTC.temperature();
    condenserOutletTemp = condenserOutletNTC.temperature();
    ambientTemp = ambientNTC.temperature();
    coolingPower = (evaporatorInletTemp - evaporatorOutletTemp) * SPECIFIC_HEAT * flowRate / 60; // Watts
    // adjust system voltage by the relative IR drop between the measured system voltage and the compressor
    powerDraw = compressorCurrent * (voltageMonitor.get12vMilliVolts() / 1000.0 - 0.0175 * compressorCurrent + 0.05 + 0.05 * coolshirtPWM.percent() / 100);

    if (analyzeNoteFrequency.available()) {
        // only log data if there's a valid result and current is > 2A
        if (analyzeNoteFrequency.validResult() && compressorCurrent > 2.0) {
            compressorFrequency = analyzeNoteFrequency.read();
            compressorFrequencyProbability = analyzeNoteFrequency.probability();            
        } else {
            compressorFrequency = 0;
            compressorFrequencyProbability = 0;
        }
    }
}

void CoolerSystem::updateFaults() {
    compressorFaultCode = compressorFault.getCode();

    // check faults and set systemFault flags
    check(systemPressure < OVERPRESSURE_THRESHOLD_KPA, SystemFault::SYSTEM_OVER_PRESSURE);
    check(coolantLevel, SystemFault::LOW_COOLANT);
    check(compressorFaultCode == CompressorFaultCode::OK, SystemFault::COMPRESSOR_FAULT);
    check(!voltageMonitor.overVoltage(), SystemFault::SYSTEM_OVERVOLT);
    check(!voltageMonitor.underVoltage(), SystemFault::SYSTEM_UNDERVOLT);

    // check flow rate after chiller pump startup period is over and verify it's sufficient
    if (chillerPumpPWM.value()) {
        int32_t pumpRunTime = millis() - pumpStartTime;
        bool flowError = instantaneousFlowRate < FLOW_RATE_MIN_THRESHOLD && pumpRunTime >= FLOW_RATE_STARTUP_TIME;
        check(!flowError, SystemFault::FLOW_RATE_LOW);
    }
}

bool CoolerSystem::updateState()
{
    switch (systemStatus) {
        case CoolerSystemStatus::STARTUP:
            if (sampleCounter < STARTUP_STABILIZATION_SAMPLES) {
                return false;
            }

            // align clock and timers to when sample stabilization is complete and we're ready
            // to process inputs for the first time
            ClockTime::setEpoch();
            updateStateTimer.reset();
            dataLogTimer.reset();
            displayInfoTimer.reset();

            systemStatus = CoolerSystemStatus::REQUIRES_RESET;
            // fall through to full state update logic, so the first update is aligned
            // with the epoch

        default:
            // this will fire on the first call, and thereafter on every interval period
            if (!updateStateTimer.check()) {
                return false;
            }

            updateCoolerData();
            updateFaults();

            // handle system flushing
            if (systemStatus != CoolerSystemStatus::FLUSH && shouldFlush) {
                // start flushing
                systemStatus = CoolerSystemStatus::FLUSH;
                pumpStartTime = millis();
                return true;
            }
            if (systemStatus == CoolerSystemStatus::FLUSH && shouldFlush && millis() < pumpStartTime + FLUSH_TIMEOUT_MS) {
                // continue flushing, ignore faults and other inputs
                return true;
            }
            if (systemStatus == CoolerSystemStatus::FLUSH) {
                // end flushing and fall through to switch handling
                systemStatus = CoolerSystemStatus::REQUIRES_RESET;
                shouldFlush = false;
            }

            if (_systemFault != (byte) SystemFault::SYSTEM_OK) {
                systemStatus = CoolerSystemStatus::REQUIRES_RESET;
            }

            CoolerSwitchPosition switchPosition = switchADC.position();
            if (switchPosition == CoolerSwitchPosition::RESET) {
                // clear system fault and return now so we don't enter REQUIRES_RESET
                _systemFault = (byte) SystemFault::SYSTEM_OK;
                compressorFaultCode = CompressorFaultCode::OK;
                compressorFault.reset();
                systemStatus = CoolerSystemStatus::RESET;
                return true;
            }

            if (systemStatus == CoolerSystemStatus::REQUIRES_RESET) {
                // don't exit this state unless switch is returned to RESET position
                return true;
            }

            switch (switchPosition) {
                case CoolerSwitchPosition::PRECHILL:
                    systemStatus = CoolerSystemStatus::PRECHILL;
                    return true;

                case CoolerSwitchPosition::PUMP_LOW:
                    systemStatus = CoolerSystemStatus::PUMP_LOW;
                    return true;

                case CoolerSwitchPosition::PUMP_MEDIUM:
                    systemStatus = CoolerSystemStatus::PUMP_MEDIUM;
                    return true;

                case CoolerSwitchPosition::PUMP_HIGH:
                    systemStatus = CoolerSystemStatus::PUMP_HIGH;
                    return true;

                default:
                    // do nothing
                    return true;
            }
    }
}

void CoolerSystem::updateOutputs()
{
    switch (systemStatus) {
        case CoolerSystemStatus::STARTUP:
            return;

        default:
            runCompressor();
            runChillerPump();
            runCoolshirtPump();
            return;
    }
}

void CoolerSystem::displayInfo()
{
#ifndef DEBUGLOG_DISABLE_LOG
    if (systemStatus == CoolerSystemStatus::STARTUP || !displayInfoTimer.check()) {
        return;
    }

    StringFormatDisplay format;

    format.formatLiteral("----------------- Cooler Statistics -------------------\n");
    format.formatLiteral("Time                             ");
    format.formatFloat3DP(ClockTime::secSinceEpoch());
    format.formatLiteral(" s\n");
    format.formatLiteral("Voltages ----------------------------------------------\n");

    format.formatLiteral("  System 12V:                    ");
    format.formatFloat3DP(voltageMonitor.get12vMilliVolts() / 1000.0);
    format.formatLiteral(" V\n");

    format.formatLiteral("  System 5V:                     ");
    format.formatFloat3DP(voltageMonitor.get5vMilliVolts() / 1000.0);
    format.formatLiteral(" V\n");

    format.formatLiteral("  System 3.3V:                   ");
    format.formatFloat3DP(voltageMonitor.get3v3MilliVolts() / 1000.0);
    format.formatLiteral(" V\n");

    format.formatLiteral("  System P3.3V:                  ");
    format.formatFloat3DP(voltageMonitor.getp3v3MilliVolts() / 1000.0);
    format.formatLiteral(" V\n");

    format.formatLiteral("Inputs ------------------------------------------------\n");

    format.formatLiteral("  Switch Position:               ");
    format.formatString(CoolerSwitchPositionToString(switchADC.position()));
    format.formatLiteral(" ( ");
    format.formatUnsignedInt(switchADC.adc());
    format.formatLiteral(" )\n");

    format.formatLiteral("  Evaporator Inlet:              ");
    format.formatFloat3DP(evaporatorInletTemp);
    format.formatLiteral(" °C ( ");
    format.formatUnsignedInt(evaporatorInletNTC.adc());
    format.formatLiteral(" )\n");

    format.formatLiteral("  Evaporator Outlet:             ");
    format.formatFloat3DP(evaporatorOutletTemp);
    format.formatLiteral(" °C ( ");
    format.formatUnsignedInt(evaporatorOutletNTC.adc());
    format.formatLiteral(" )\n");

    format.formatLiteral("  Condenser Inlet:               ");
    format.formatFloat3DP(condenserInletTemp);
    format.formatLiteral(" °C ( ");
    format.formatUnsignedInt(condenserInletNTC.adc());
    format.formatLiteral(" )\n");

    format.formatLiteral("  Condenser Outlet:              ");
    format.formatFloat3DP(condenserOutletTemp);
    format.formatLiteral(" °C ( ");
    format.formatUnsignedInt(condenserOutletNTC.adc());
    format.formatLiteral(" )\n");

    format.formatLiteral("  Ambient:                       ");
    format.formatFloat3DP(ambientTemp);
    format.formatLiteral(" °C ( ");
    format.formatUnsignedInt(ambientNTC.adc());
    format.formatLiteral(" )\n");

    format.formatLiteral("  Coolant Level:                 ");
    coolantLevel ? format.formatLiteral("OK\n") : format.formatLiteral("LOW\n");

    format.formatLiteral("  Flow Rate:                     ");
    format.formatFloat3DP(flowRate);
    format.formatLiteral(" L/min\n");

    format.formatLiteral("  Compressor Current:            ");
    format.formatFloat3DP(compressorCurrent);
    format.formatLiteral(" A ( ");
    format.formatUnsignedInt(currentSensor.adc());
    format.formatLiteral(" )\n");

    format.formatLiteral("  System Pressure:               ");
    format.formatUnsignedInt(systemPressure);
    format.formatLiteral(" kPa ( ");
    format.formatUnsignedInt(pressureSensor.adc());
    format.formatLiteral(" )\n");

    format.formatLiteral("State -------------------------------------------------\n");

    format.formatLiteral("  System Status:                 ");
    format.formatString(CoolerSystemStatusToString(systemStatus));
    format.formatLiteral("\n");

    format.formatLiteral("  Chiller:                       ");
    systemEnableOutput.value() ? format.formatLiteral("ON\n") : format.formatLiteral("OFF\n");

    format.formatLiteral("  Coolshirt Pump:                ");
    coolshirtPWM.value() ? format.formatLiteral("ON\n") : format.formatLiteral("OFF\n");

    format.formatLiteral("  Chiller Pump Speed:            ");
    format.formatFloat3DP(chillerPumpSpeed);
    format.formatLiteral(" V\n");

    format.formatLiteral("  Compressor Speed:              ");
    format.formatUnsignedInt(roundf(compressorSpeed * 100));
    format.formatLiteral(" %\n");

    format.formatLiteral("  Undertemp Cutoff:              ");
    undertempCutoff ? format.formatLiteral("CUTOFF\n") : format.formatLiteral("OK\n");

    format.formatLiteral("  Cooling Power:                 ");
    format.formatFloat3DP(coolingPower);
    format.formatLiteral(" W\n");

    format.formatLiteral("  Power Draw:                    ");
    format.formatFloat3DP(compressorCurrent * voltageMonitor.get12vMilliVolts() / 1000.0);
    format.formatLiteral(" W\n");

    if (compressorFrequency) {
        format.formatLiteral("  Compressor Frequency:          ");
        format.formatFloat3DP(compressorFrequency);
        format.formatLiteral(" Hz\n");
        format.formatLiteral("  Frequency Probability:         ");
        format.formatFloat3DP(compressorFrequencyProbability);
        format.formatLiteral("\n");
    } else {
        format.formatLiteral("  Compressor Frequency:          ---\n");
        format.formatLiteral("  Frequency Probability:         ---\n");
    }

    format.formatLiteral("Faults ------------------------------------------------\n");
    printFaultLine(format, SystemFault::LOW_COOLANT, _systemFault);
    printFaultLine(format, SystemFault::FLOW_RATE_LOW, _systemFault);
    printFaultLine(format, SystemFault::SYSTEM_OVER_PRESSURE, _systemFault);
    printFaultLine(format, SystemFault::SYSTEM_UNDERVOLT, _systemFault);
    printFaultLine(format, SystemFault::SYSTEM_OVERVOLT, _systemFault);
    printFaultLine(format, SystemFault::COMPRESSOR_FAULT, _systemFault);
    format.formatLiteral("  Compressor Fault Code:         ");
    format.formatString(CompressorFaultToString(compressorFaultCode));
    if (compressorFaultCode != CompressorFaultCode::OK) {
        format.formatLiteral(" (");
        format.formatFloat3DP(compressorFault.durationSinceFaultRecorded() / 1000.0);
        format.formatLiteral(" s ago)");
    }
    format.formatLiteral("\n");
    format.formatLiteral("-------------------------------------------------------");

    Serial.write(format.finish(), format.length());
#endif
}

void CoolerSystem::loop()
{
    if (!msTick.check()) {
        return;
    }

    acquireSamples();    
    
    // sanity check in case of logic bugs or unexpected system conditions: if evaporator outlet temp drops below 0C,
    // panic and shut the compressor down
    if (evaporatorOutletNTC.temperature() <= EVAPORATOR_OUTLET_PANIC_TEMPERATURE && systemEnableOutput.value()) {
        undertempCutoff = true;
        systemStatus = CoolerSystemStatus::REQUIRES_RESET;
        shutdownCompressor();
        LOG_ERROR("Panic condition: evaporator outlet temp", evaporatorOutletNTC.temperature(), "C (below", EVAPORATOR_OUTLET_PANIC_TEMPERATURE, "), shutting down compressor");
        return;
    }

    CoolerSystemStatus prevStatus = systemStatus;
    bool stateComputed = updateState();
    if (systemStatus != prevStatus) {
        LOG_INFO("[", ClockTime::secSinceEpoch(), " s] status changed from", CoolerSystemStatusToString(prevStatus), "to", CoolerSystemStatusToString(systemStatus));
    }

    if (systemStatus != CoolerSystemStatus::STARTUP) {
#if NTC_DEBUG
        sampleLogger.logSamples(ClockTime::millisSinceEpoch(), evaporatorInletNTC.latest(), evaporatorOutletNTC.latest(), 0, 0);
        //sampleLogger.logSamples(sampleTime, currentSensor.latest(), compressorCurrentBiquadOutput + 30000, analogRead(ADC_SYSTEM_12V), 0);
        //sampleLogger.logSamples(sampleTime, currentSensor.latest(), compressorCurrentBiquadOutput + 30000, analyzeNoteFrequency.read() * 100, analyzeNoteFrequency.probability() * 1000);
#elif FLOW_DEBUG
        if (flowSensor.lastPulseIndex() != lastLoggedFlowPulse) {
            sampleLogger.logSamples(ClockTime::millisSinceEpoch(), flowSensor.lastPulseIndex(), flowSensor.lastPulseDuration(), chillerPumpPWM.value(), coolantLevelBounce.read());
            LOG_INFO("[", ClockTime::secSinceEpoch(), " s] pulse", flowSensor.lastPulseIndex(), ", duration", flowSensor.lastPulseDuration(), ", flow rate", flowSensor.instantaneousFlowRate(), ", speed", chillerPumpSpeed);
            lastLoggedFlowPulse = flowSensor.lastPulseIndex();
        }
#endif
    }

    if (stateComputed) {
        updateOutputs();
    }

    logData();
    displayInfo();
};

void CoolerSystem::getSystemData(CoolerSystemData &data) {
    data.systemStatus = systemStatus;
    data.evaporatorInletTemp = evaporatorInletTemp;
    data.evaporatorOutletTemp = evaporatorOutletTemp;
    data.condenserInletTemp = condenserInletTemp;
    data.condenserOutletTemp = condenserOutletTemp;
    data.ambientTemp = ambientTemp;
    data.compressorSpeed = compressorSpeed;
    data.coolantLevel = coolantLevel;
    data.fault = _systemFault;
    data.compressorFaultCode = compressorFaultCode;
    data.flowRate = flowRate;
    data.systemPressure = systemPressure;
    data.compressorCurrent = compressorCurrent;
    data.compressorFrequency = compressorFrequency;
};

uint32_t CoolerSystem::lastFlowPulseMicros() {
    return flowSensor.lastPulseMicros();
}

void CoolerSystem::setCompressorSpeed(uint32_t speed) {
    if (speed == 0) {
        compressorSpeed = 0;
    } else if (speed >= 1 && speed <= 7) {
        compressorSpeed = CompressorSpeeds[speed - 1];
    } else {
        return;
    }

    analogWrite(compressorSpeedPin, compressorSpeed * COMPRESSOR_SPEED_RATIO_TO_ANALOG);
    LOG_INFO("Setting compressor speed to", roundf(compressorSpeed * 100), "%");
}

void CoolerSystem::setCompressorSpeedPercentOffset(int32_t offset) {
    compressorSpeed = roundf(compressorSpeed * 100 + offset) / 100;
    analogWrite(compressorSpeedPin, compressorSpeed * COMPRESSOR_SPEED_RATIO_TO_ANALOG);
    LOG_INFO("Setting compressor speed to", roundf(compressorSpeed * 100), "%");
}

void CoolerSystem::toggleFlush() {
    shouldFlush = systemStatus != CoolerSystemStatus::FLUSH;
}

int32_t clampAndScale(float val, int32_t minVal, int32_t maxVal, uint32_t scale) {
    return max(min(roundf(val * scale), maxVal), minVal);
}

void CoolerSystem::getCANMessage(CAN_message_t& msg)
{
    msg.id = CANID_COOLER_SYSTEM;
    msg.ext = false;
    msg.len = 6;
    msg.timeout = 1; // ms

    // byte | purpose
    // ------------------------------
    // 0,1  | chiller reservoir temp (int16_t) = T (degrees C) * 256
    //        byte 0: integer temp in C, byte 1: fractional temp * 256
    // 2    | compressor speed (uint8_t) = speed (%) (0 - 100)
    // 3    | state bitmap
    //      |   [6]: chiller active
    //      |   [5]: chiller pump active
    //      |   [4]: coolshirt pump active
    //      |   [3]: under-temp compressor cut-off
    //      |   [2,1,0]: system status
    // 4    | system faults
    // 5    | compressor fault

    uint16_t temp = clampAndScale(evaporatorInletTemp, 0, UINT16_MAX, 256);
    msg.buf[0] = (temp & 0xff00) >> 8;
    msg.buf[1] = temp & 0xff;

    msg.buf[2] = clampAndScale(compressorSpeed, 0, 100, 100);

    uint8_t state = 0;
    systemEnableOutput.value() && (state |= 1 << 6);
    chillerPumpPWM.value() && (state |= 1 << 5);
    coolshirtPWM.value() && (state |= 1 << 4);
    undertempCutoff && (state |= 1 << 3);
    state |= (uint8_t) systemStatus & 0x07;
    msg.buf[3] = state;

    msg.buf[4] = (uint8_t) _systemFault;
    msg.buf[5] = (uint8_t) compressorFaultCode;

    msg.buf[6] = 0;
    msg.buf[7] = 0;
};

void CoolerSystem::logData() {
    if (systemStatus == CoolerSystemStatus::STARTUP || !dataLogTimer.check()) {
        return;
    }

    StringFormatLog format;
    getLogMessage(format);
    DataSDLogger::logData(format.finish(), format.length());

    // uint32_t m = micros();
    // CAN_message_t message;
    // getCANMessage(message);
    // //CANBus.write(message);
    // LOG_INFO("CANBus write took", micros()-m, "us");
}

const char* CoolerSystem::getLogHeader() {
    return "time,evapInletTemp,evapOutletTemp,condInletTemp,condOutletTemp,ambientTemp,evapInletTempStdev,flowRate,instantaneousFlowRate,pressure,compressorCurrent,compressorFrequency,compressorFrequencyProbability,coolantLevel,12v,5v,3v3,p3v3,coolingPower,powerDraw,switchPos,switchADC,status,systemEnable,chillerPumpSpeed,coolshirtEnable,compressorSpeed,underTempCutoff,systemFault,compressorFault,acquiredSamples\n";
}

void CoolerSystem::getLogMessage(StringFormatLog& format)
{
    format.formatFloat3DP(ClockTime::secSinceEpoch());
    format.formatFloat3DP(evaporatorInletTemp);
    format.formatFloat3DP(evaporatorOutletTemp);
    format.formatFloat3DP(condenserInletTemp);
    format.formatFloat3DP(condenserOutletTemp);
    format.formatFloat3DP(ambientTemp);
    format.formatFloat3DP(evaporatorInletNTC.stdev());
    format.formatFloat3DP(flowRate);
    format.formatFloat3DP(instantaneousFlowRate);
    format.formatUnsignedInt(systemPressure);
    format.formatFloat3DP(compressorCurrent);

    // only log compressor frequency if there's a valid result
    if (compressorFrequency) {
        format.formatFloat3DP(compressorFrequency);
        format.formatFloat3DP(compressorFrequencyProbability);
    } else {
        format.formatLiteral("");
        format.formatLiteral("");
    }

    format.formatBool(coolantLevel);
    format.formatFloat3DP(voltageMonitor.get12vMilliVolts() / 1000.0);
    format.formatFloat3DP(voltageMonitor.get5vMilliVolts() / 1000.0);
    format.formatFloat3DP(voltageMonitor.get3v3MilliVolts() / 1000.0);
    format.formatFloat3DP(voltageMonitor.getp3v3MilliVolts() / 1000.0);
    format.formatFloat3DP(coolingPower);
    format.formatFloat3DP(powerDraw);
    format.formatInt((int32_t) switchADC.position());
    format.formatUnsignedInt(switchADC.adc());
    format.formatInt((int32_t) systemStatus);
    format.formatBool(systemEnableOutput.value());
    format.formatFloat3DP(chillerPumpSpeed);
    format.formatBool(coolshirtPWM.value());
    format.formatFloat3DP(compressorSpeed);
    format.formatBool(undertempCutoff);
    format.formatBinary(_systemFault);
    format.formatUnsignedInt((uint32_t) compressorFaultCode);
    format.formatUnsignedInt(sampleCounter - loggedSampleCounter);

    loggedSampleCounter = sampleCounter;
};
