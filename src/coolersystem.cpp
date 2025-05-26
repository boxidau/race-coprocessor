#include "coolersystem.h"
#include "watchdog.h"
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
    chillerLoop.setup(UPDATE_STATE_TIMER_MS);
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

    voltageMonitor.setup();

#if ANF_SAMPLES_TEST
    analyzeNoteFrequency.begin();
#endif
}

void CoolerSystem::setupLogging()
{
#if NTC_DEBUG
    sampleLogger.ensureSetup("time,12v,5v,3v3");
    //sampleLogger.ensureSetup("time,current,biquad,12V");
    // sampleLogger.ensureSetup("time,inlet,outlet");
#elif FLOW_DEBUG
    sampleLogger.ensureSetup("time,index,duration,inletTemp,outletTemp,chillerPumpSpeed,coolantLevel,flowRate");
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
    int32_t latest12v = voltageMonitor.getLatest12vMilliVolts();
    // avoid divide by zero
    if (latest12v == 0) {
        return;
    }

    float pumpSpeed = systemStatus == CoolerSystemStatus::FLUSH ?
        CHILLER_PUMP_DEFAULT_SPEED :
        chillerLoop.getPumpSpeed();
    chillerPumpPWM.set(round(pumpSpeed / latest12v * 1000 * ADC_MAX));
}

void CoolerSystem::runCompressor()
{
    // reset watchdog at the point we handle compressor control
    watchdog_reset();

    float speed = chillerLoop.getCompressorSpeedQuantized();
    float enableCompressor = speed > 0;
    if (systemEnableOutput.value() && !enableCompressor) {
        analyzeNoteFrequency.stop();
        compressorFrequency = 0;
        compressorFrequencyProbability = 0;
        vfRatio = 0;
    } else if (!systemEnableOutput.value() && enableCompressor) {
        // reset note frequency analyzer to start with fresh data
        analyzeNoteFrequency.begin();
    }
    systemEnableOutput.setBoolean(enableCompressor);

    if (!compressorSpeedOverride) {
        analogWrite(compressorSpeedPin, chillerLoop.getCompressorSpeedVoltage());
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

    // update the temperature the flow sensor is seeing, so it can calibrate
    flowSensor.updateTemp(evaporatorOutletTemp);

    // water flowing through the evaporator has a lag time depending on flow rate.
    // in order to calculate cooling power correctly, we need to compare the outlet temp
    // against the inlet temp from that lag time ago. the lag time is approximately 1.6 sec
    // (16 samples) at 3.4Lpm.
    evaporatorInletTempSamples.push(evaporatorInletTemp);
    if (flowRate >= FLOW_RATE_MIN_THRESHOLD) {
        float lagSamples = EVAPORATOR_VOLUME * 60 * 1000 / (flowRate * UPDATE_STATE_TIMER_MS);
        uint32_t lowerOffset = floor(lagSamples);
        uint32_t upperOffset = ceil(lagSamples);
        if (evaporatorInletTempSamples.hasSampleAt(-upperOffset)) {
            // interpolate samples
            float lowerSample = evaporatorInletTempSamples[-lowerOffset];
            float upperSample = evaporatorInletTempSamples[-upperOffset];
            float inletTempLagged = (upperSample - lowerSample) * (lagSamples - lowerOffset) + lowerSample;
            coolingPower = (inletTempLagged - evaporatorOutletTemp) * SPECIFIC_HEAT * DENSITY * flowRate / 60; // Watts
        } else {
            coolingPower = 0;
        }
    } else {
        coolingPower = 0;
    }

    // adjust system voltage by the relative IR drop between the measured system voltage and the compressor
    float compressorVoltage = (voltageMonitor.get12vMilliVolts() / 1000.0 - 0.0175 * compressorCurrent + 0.05 + 0.05 * coolshirtPWM.percent() / 100);
    powerDraw = compressorCurrent * compressorVoltage;

    if (analyzeNoteFrequency.available()) {
        // only log data if there's a valid result and current is > 3A
        if (analyzeNoteFrequency.validResult() && compressorCurrent > 3.0) {
            compressorFrequency = analyzeNoteFrequency.read();
            compressorFrequencyProbability = analyzeNoteFrequency.probability();
            vfRatio = compressorFrequency ? compressorVoltage * chillerLoop.getCompressorSpeedQuantized() / compressorFrequency : 0;
        } else {
            compressorFrequency = 0;
            compressorFrequencyProbability = 0;
            vfRatio = 0;
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
        uint32_t pumpRunTime = chillerLoop.getMillisSincePumpStart();
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

            // set fault code, this will force us into a REQUIRES_RESET state below
            check(false, SystemFault::RESET_ON_STARTUP);

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
                flushStartTime = millis();
                return true;
            }
            if (systemStatus == CoolerSystemStatus::FLUSH && shouldFlush && millis() < flushStartTime + FLUSH_TIMEOUT_MS) {
                // continue flushing, ignore faults and other inputs
                return true;
            }
            if (systemStatus == CoolerSystemStatus::FLUSH) {
                // end flushing and fall through to switch handling
                check(false, SystemFault::RESET_ON_STARTUP);
                shouldFlush = false;
            }

            if (_systemFault != (byte) SystemFault::SYSTEM_OK) {
                systemStatus = CoolerSystemStatus::REQUIRES_RESET;
            }

            CoolerSwitchPosition switchPosition = overrideSwitchPosition == CoolerSwitchPosition::UNKNOWN ? switchADC.position() : overrideSwitchPosition;
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

void CoolerSystem::updateChillerLoopState()
{
    bool systemEnableRequested = false;
    switch (systemStatus) {
        case CoolerSystemStatus::PRECHILL:
        case CoolerSystemStatus::PUMP_LOW:
        case CoolerSystemStatus::PUMP_MEDIUM:
        case CoolerSystemStatus::PUMP_HIGH:
            systemEnableRequested = true;
            break;

        default:
            break;
    }

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
            break;
    }

    chillerLoop.updateState(systemEnableRequested, evaporatorInletTemp, restartTemp, cutoffTemp, evaporatorOutletTemp, instantaneousFlowRate);    
}

void CoolerSystem::displayInfo()
{
#if RC_DEBUG
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
    CoolerSwitchPosition switchPosition = overrideSwitchPosition == CoolerSwitchPosition::UNKNOWN ? switchADC.position() : overrideSwitchPosition;
    format.formatString(CoolerSwitchPositionToString(switchPosition));
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

    format.formatLiteral("  Coolshirt Pump:                ");
    coolshirtPWM.value() ? format.formatLiteral("ON\n") : format.formatLiteral("OFF\n");

    format.formatLiteral("  Chiller Loop State:            ");
    format.formatString(chillerLoop.getStateString());
    format.formatLiteral("\n");

    format.formatLiteral("  Chiller Pump Speed:            ");
    float pumpSpeed = systemStatus == CoolerSystemStatus::FLUSH ?
        CHILLER_PUMP_DEFAULT_SPEED :
        chillerLoop.getPumpSpeed();
    format.formatFloat3DP(pumpSpeed);
    format.formatLiteral(" V\n");

    format.formatLiteral("  Compressor Speed, Continuous:  ");
    float compressorSpeedContinuous = compressorManualControl ?
        compressorSpeedOverride :
        chillerLoop.getCompressorSpeedContinuous();
    format.formatUnsignedInt(round(compressorSpeedContinuous * 100));
    format.formatLiteral(" %\n");

    format.formatLiteral("  Compressor Speed, Quantized:   ");
    float compressorSpeedQuantized = compressorManualControl ?
        compressorSpeedOverride :
        chillerLoop.getCompressorSpeedQuantized();
    format.formatUnsignedInt(round(compressorSpeedQuantized * 100));
    format.formatLiteral(" %\n");

    format.formatLiteral("  Compressor Cooldown Time:      ");
    uint32_t cooldownTime = chillerLoop.getCompressorCooldownSecondsRemaining();
    if (cooldownTime) {
        format.formatUnsignedInt(cooldownTime);
        format.formatLiteral(" s\n");
    } else {
        format.formatLiteral("---\n");
    }

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
    printFaultLine(format, SystemFault::RESET_ON_STARTUP, _systemFault);
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

#if 0 // RC_DEBUG
    if (voltageMonitor.isKillswitchOff()) {
        // immediately flush logs, and throw a fault
        DataSDLogger::finish();
    }

    static bool alternatorOn;
    if (voltageMonitor.get12vMilliVolts() > 13000) {
        alternatorOn = true;
    }

    if (voltageMonitor.getLatest12vMilliVolts() < 12000) {
        uint32_t m = micros();
        while (voltageMonitor.getLatest12vMilliVolts() < 12200 && voltageMonitor.getLatest12vMilliVolts() > 2000) {
            voltageMonitor.loop();
            LOG_INFO("Time", micros() - m, "us, 12V", voltageMonitor.getLatest12vMilliVolts(), "5V", voltageMonitor.getLatest5vMilliVolts(), "3.3V", voltageMonitor.getLatest3v3MilliVolts());
        }
    }
#endif

    CoolerSystemStatus prevStatus = systemStatus;
    bool stateComputed = updateState();
    if (systemStatus != prevStatus) {
        LOG_INFO("[", ClockTime::secSinceEpoch(), " s] status changed from", CoolerSystemStatusToString(prevStatus), "to", CoolerSystemStatusToString(systemStatus));
    }

    if (systemStatus != CoolerSystemStatus::STARTUP) {
#if NTC_DEBUG
        //sampleLogger.logSamples(ClockTime::millisSinceEpoch(), voltageMonitor.getLatest12vMillivolts(), voltageMonitor.getLatest5vMillivolts(), voltageMonitor.getLatest3v3Millivolts(), 0);
        // sampleLogger.logSamples(ClockTime::millisSinceEpoch(), evaporatorInletNTC.latest(), evaporatorOutletNTC.latest(), 0, 0);
        sampleLogger.logSamples(ClockTime::millisSinceEpoch(), currentSensor.latest(), 0, 0, 0, 0, 0, 0, 0);
        //sampleLogger.logSamples(sampleTime, currentSensor.latest(), compressorCurrentBiquadOutput + 30000, analogRead(ADC_SYSTEM_12V), 0);
        //sampleLogger.logSamples(sampleTime, currentSensor.latest(), compressorCurrentBiquadOutput + 30000, analyzeNoteFrequency.read() * 100, analyzeNoteFrequency.probability() * 1000);
#elif FLOW_DEBUG
        if (flowSensor.lastPulseIndex() != lastLoggedFlowPulse) {
            sampleLogger.logSamples(ClockTime::millisSinceEpoch(), flowSensor.lastPulseIndex(), flowSensor.lastPulseDuration(), round(evaporatorInletNTC.temperatureFor(evaporatorInletNTC.latest()) * 1000), round(evaporatorOutletNTC.temperatureFor(evaporatorOutletNTC.latest()) * 1000), round(chillerPumpSpeed * 1000), coolantLevelBounce.read(), flowSensor.flowRate() * 1000, 0);
            //LOG_INFO("[", ClockTime::secSinceEpoch(), " s] pulse", flowSensor.lastPulseIndex(), ", duration", flowSensor.lastPulseDuration(), ", flow rate", flowSensor.instantaneousFlowRate(), ", speed", chillerPumpSpeed);
            lastLoggedFlowPulse = flowSensor.lastPulseIndex();
        }
#endif
    }

    if (stateComputed) {
        updateChillerLoopState();
        runCompressor();
        runCoolshirtPump();
    }
    // update chiller pump on every tick because system voltage can change quickly,
    // and we want the pump voltage to remain stable
    runChillerPump();

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
    data.compressorSpeed = chillerLoop.getCompressorSpeedQuantized();
    data.coolantLevel = coolantLevel;
    data.fault = _systemFault;
    data.compressorFaultCode = compressorFaultCode;
    data.flowRate = flowRate;
    data.systemPressure = systemPressure;
    data.compressorCurrent = compressorCurrent;
    data.compressorFrequency = compressorFrequency;
}

uint32_t CoolerSystem::lastFlowPulseMicros() {
    return flowSensor.lastPulseMicros();
}

void CoolerSystem::setCompressorSpeed(uint32_t speed) {
    compressorManualControl = true;

    if (speed == 0) {
        compressorSpeedOverride = 0;
    } else if (speed <= NUM_COMPRESSOR_SPEEDS) {
        compressorSpeedOverride = CompressorSpeedVoltageRatios[speed - 1];
    } else {
        return;
    }

    analogWrite(compressorSpeedPin, compressorSpeedOverride * COMPRESSOR_SPEED_RATIO_TO_ANALOG);
    LOG_INFO("Setting compressor speed voltage ratio to", round(compressorSpeedOverride * 100), "%");
}

void CoolerSystem::setCompressorSpeedPercentOffset(int32_t offset) {
    compressorManualControl = true;
    compressorSpeedOverride = round(compressorSpeedOverride * 100 + offset) / 100;
    analogWrite(compressorSpeedPin, compressorSpeedOverride * COMPRESSOR_SPEED_RATIO_TO_ANALOG);
    LOG_INFO("Setting compressor speed voltage ratio to", round(compressorSpeedOverride * 100), "%");
}

void CoolerSystem::resumeCompressorControl() {
    compressorManualControl = false;
}

void CoolerSystem::toggleSwitchPosition(CoolerSwitchPosition position) {
    if (overrideSwitchPosition == position) {
        overrideSwitchPosition = CoolerSwitchPosition::UNKNOWN;
        return;
    }

    overrideSwitchPosition = position;
}

void CoolerSystem::toggleFlush() {
    shouldFlush = systemStatus != CoolerSystemStatus::FLUSH;
}

int32_t clampAndScale(float val, int32_t minVal, int32_t maxVal, float scale) {
    return max(min(round(val * scale), maxVal), minVal);
}

void CoolerSystem::getCANMessage(CAN_message_t& msg)
{
    msg.id = CANID_COOLER_SYSTEM;
    msg.ext = false;
    msg.len = 8;
    msg.timeout = 0; // ms

    // byte | purpose
    // ------------------------------
    // 0,1  | evaporator inlet temp (int16_t) = T (degrees C) * 256
    //        byte 0: integer temp in C, byte 1: fractional temp * 256
    // 2    | quantized compressor speed (uint8_t) = speed (%) (0 - 100)
    // 3    | state bitmap
    //      |   [7]: compressor active
    //      |   [6]: chiller pump active
    //      |   [5]: coolshirt pump active
    //      |   [4,3]: chiller loop status
    //      |   [2,1,0]: system status
    // 4    | system faults
    // 5    | compressor fault
    // 6    | [6,5,4]: dimmer period, 1 .. 4 (1 = 100% brightness, 2 = 50%, 3 = 33%, 4 = 25%)
    //      | [3,2,1,0]: fault blinks for display purposes
    // 7    | cooling power (int8_t) = P (W) / 8

    uint16_t temp = clampAndScale(evaporatorInletTemp, 0, UINT16_MAX, 256);
    msg.buf[0] = (temp & 0xff00) >> 8;
    msg.buf[1] = temp & 0xff;

    msg.buf[2] = clampAndScale(chillerLoop.getCompressorSpeedQuantized(), 0, 100, 100);

    uint8_t state = 0;
    systemEnableOutput.value() && (state |= 1 << 7);
    chillerPumpPWM.value() && (state |= 1 << 6);
    coolshirtPWM.value() && (state |= 1 << 5);
    state |= (uint8_t) chillerLoop.getState() & 0x03 << 3;
    state |= (uint8_t) systemStatus & 0x07;
    msg.buf[3] = state;

    msg.buf[4] = (uint8_t) _systemFault;
    msg.buf[5] = (uint8_t) compressorFaultCode;

    msg.buf[6] = ((uint8_t) getFaultBlinks(_systemFault, compressorFaultCode)) & 0x0f;
    float dimmerPeriod = 100 / (float) ClockTime::getDimmerBrightnessPercent();
    msg.buf[6] |= ((uint8_t) clampAndScale(dimmerPeriod, 1, 4, 1)) << 4;

    msg.buf[7] = (int8_t) clampAndScale(coolingPower, INT8_MIN, INT8_MAX, 0.125);
};

void CoolerSystem::setLapCount(uint8_t lapCountInput) {
    lapCount = lapCountInput;
    LOG_INFO("Received lap count", lapCount);
}

void CoolerSystem::logData() {
    if (systemStatus == CoolerSystemStatus::STARTUP || !dataLogTimer.check()) {
        return;
    }

    StringFormatLog format;
    getLogMessage(format);
    DataSDLogger::logData(format.finish(), format.length());

    uint32_t m = micros();
    CAN_message_t message;
    getCANMessage(message);
    CANBus.write(message);
    LOG_INFO("CANBus write took", micros() - m, "us");
}

const char* CoolerSystem::getLogHeader() {
    return "time,evapInletTemp,evapOutletTemp,condInletTemp,condOutletTemp,ambientTemp,evapInletTempStdev,flowRate,instantaneousFlowRate,pressure,compressorCurrent,compressorFrequency,compressorFrequencyProbability,vfRatio,12v,5v,3v3,p3v3,coolingPower,powerDraw,switchPos,switchADC,status,coolshirtEnable,chillerLoopState,chillerPumpSpeed,evapInletTempFiltered,compressorSpeedContinuous,compressorSpeedQuantized,lowCoolant,flowRateLow,overPressure,underVolt,overVolt,compressorFault,lapCount,acquiredSamples\n";
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
        format.formatFloat3DP(vfRatio);
    } else {
        format.formatLiteral("");
        format.formatLiteral("");
        format.formatLiteral("");
    }

    format.formatFloat3DP(voltageMonitor.get12vMilliVolts() / 1000.0);
    format.formatFloat3DP(voltageMonitor.get5vMilliVolts() / 1000.0);
    format.formatFloat3DP(voltageMonitor.get3v3MilliVolts() / 1000.0);
    format.formatFloat3DP(voltageMonitor.getp3v3MilliVolts() / 1000.0);
    format.formatFloat3DP(coolingPower);
    format.formatFloat3DP(powerDraw);
    format.formatInt((int32_t) switchADC.position());
    format.formatUnsignedInt(switchADC.adc());
    format.formatInt((int32_t) systemStatus);
    format.formatBool(coolshirtPWM.value());
    format.formatUnsignedInt((uint32_t) chillerLoop.getState());
    format.formatFloat3DP(chillerLoop.getPumpSpeed());
    format.formatFloat3DP(chillerLoop.getEvapInletTempFiltered());
    format.formatFloat3DP(chillerLoop.getCompressorSpeedContinuous());
    format.formatFloat3DP(chillerLoop.getCompressorSpeedQuantized());
    format.formatBool(_systemFault & (byte) SystemFault::LOW_COOLANT);
    format.formatBool(_systemFault & (byte) SystemFault::FLOW_RATE_LOW);
    format.formatBool(_systemFault & (byte) SystemFault::SYSTEM_OVER_PRESSURE);
    format.formatBool(_systemFault & (byte) SystemFault::SYSTEM_UNDERVOLT);
    format.formatBool(_systemFault & (byte) SystemFault::SYSTEM_OVERVOLT);
    format.formatUnsignedInt((uint32_t) compressorFaultCode);
    format.formatUnsignedInt(lapCount);
    format.formatUnsignedInt(sampleCounter - loggedSampleCounter);

    loggedSampleCounter = sampleCounter;
};

uint32_t CoolerSystem::getFaultBlinks(byte fault, CompressorFaultCode compressorFaultCode) {
    if (fault & (byte) SystemFault::RESET_ON_STARTUP) {
        return 1;
    }

    if (fault & (byte) SystemFault::LOW_COOLANT) {
        return 2;
    }

    if (fault & (byte) SystemFault::FLOW_RATE_LOW) {
        return 3;
    }

    if (fault & (byte) SystemFault::SYSTEM_OVER_PRESSURE) {
        return 4;
    }

    if (fault & (byte) SystemFault::SYSTEM_UNDERVOLT) {
        return 5;
    }

    if (fault & (byte) SystemFault::SYSTEM_OVERVOLT) {
        return 6;
    }

    if (fault & (byte) SystemFault::COMPRESSOR_FAULT) {
        if (compressorFaultCode == CompressorFaultCode::HIGH_CURRENT) {
            return 7;
        }

        if (compressorFaultCode == CompressorFaultCode::MOTOR_BLOCKED) {
            return 8;
        }

        if (compressorFaultCode == CompressorFaultCode::UNDER_VOLTAGE) {
            return 9;
        }

        if (compressorFaultCode == CompressorFaultCode::FAN_FAILURE) {
            return 10;
        }

        if (compressorFaultCode == CompressorFaultCode::COMPRESSOR_OFFLINE) {
            return 11;
        }

        if (compressorFaultCode == CompressorFaultCode::COMPRESSOR_OVERHEAT) {
            return 12;
        }

        if (compressorFaultCode == CompressorFaultCode::SYSTEM_OVERPRESSURE) {
            return 13;
        }
    }

    return fault ? 14 : 0;
}