#include "coolersystem.h"
#include "clocktime.h"

void printFaultLine(StringFormatCSV& format, SystemFault f, byte systemFault) {
    const char* faultName = SystemFaultToString(f);
    size_t faultLen = strlen(faultName);
    size_t prefLength = faultLen + ((uint32_t) f > 10 ? 2 : 1);
    format.formatLiteral("  ");
    format.formatString(faultName, faultLen);
    format.formatLiteral(" ( ");
    format.formatUnsignedInt((uint32_t) f);
    format.formatString(" ):             ", max(28 - prefLength, 0));
    systemFault & (byte)f ? format.formatLiteral("ALARM\n") : format.formatLiteral("OK\n");
}

void CoolerSystem::setup()
{
    // setup pin modes and initial states
    systemEnableOutput.setup();
    coolshirtPWM.setup();
    chillerPumpPWM.setup();

    flowSensor.setup();
    pinMode(coolantLevelPin, INPUT);
    compressorFault.setup();

    evaporatorInletNTC.setup();
    evaporatorInletA10.setup();
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

    voltageMonitor.setup();
}

void CoolerSystem::pollCoolantLevel()
{
    if (coolantLevelBounce.update()) {
        coolantLevel = coolantLevelBounce.read();
    }
}

void CoolerSystem::runChillerPump()
{
    switch (systemStatus) {
        case CoolerSystemStatus::PRECHILL:
        case CoolerSystemStatus::PUMP_LOW:
        case CoolerSystemStatus::PUMP_MEDIUM:
        case CoolerSystemStatus::PUMP_HIGH:
        case CoolerSystemStatus::FLUSH:
            if (chillerPumpPWM.value() == 0) {
                pumpStartTime = millis();
                chillerPumpPWM.setPercent(50);
            }
            return;

        default:
            chillerPumpPWM.set(0);
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
            cutoffTemp = COMPRESSOR_UNDER_TEMP_CUTOFF_MED;
            restartTemp = COMPRESSOR_RESTART_TEMP_MED;
            break;            
        case CoolerSystemStatus::PUMP_HIGH:
        case CoolerSystemStatus::PRECHILL:
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
    if (evaporatorInletA10Temp < cutoffTemp) {
        undertempCutoff = true;
    }
    if (evaporatorInletA10Temp >= restartTemp) {
        undertempCutoff = false;
    }

    if (undertempCutoff) {
        shutdownCompressor();
        return;
    }
    
    startupCompressor();
}

void CoolerSystem::shutdownCompressor()
{
    if (systemEnableOutput.value()) {
        compressorShutoffTime = millis();
        systemEnableOutput.setBoolean(false);
        analogWrite(compressorSpeedPin, 0);
        compressorPID.SetMode(MANUAL);
    }
}

void CoolerSystem::startupCompressor()
{
    if (!systemEnableOutput.value() &&
        (!compressorShutoffTime || millis() >= compressorShutoffTime + COMPRESSOR_MIN_COOLDOWN_MS)) {
        systemEnableOutput.setBoolean(true);
        compressorSpeed = COMPRESSOR_DEFAULT_SPEED;
        //compressorPID.SetMode(AUTOMATIC);
        analogWrite(compressorSpeedPin, compressorSpeed * COMPRESSOR_SPEED_RATIO_TO_ANALOG);
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

bool CoolerSystem::hasStarted()
{
    return systemStatus != CoolerSystemStatus::STARTUP;
}

void CoolerSystem::acquireSamples()
{
    // sample evap inlet after outlet, for reduced noise
    evaporatorOutletNTC.loop();
    evaporatorInletNTC.loop();
    evaporatorInletA10.loop();
    condenserInletNTC.loop();
    condenserOutletNTC.loop();
    ambientNTC.loop();

    pressureSensor.loop();
    currentSensor.loop();
    compressorFault.loop();
    switchADC.loop();
    voltageMonitor.loop();

    pollCoolantLevel();

    sampleCounter++;
}

void CoolerSystem::updateCoolerData() {
    flowRate = flowSensor.flowRate();
    systemPressure = pressureSensor.calibratedValue();
    compressorCurrent = currentSensor.calibratedValue() / 1000; // mA -> A
    evaporatorInletTemp = evaporatorInletNTC.temperature();
    evaporatorInletA10Temp = evaporatorInletA10.temperature();
    evaporatorOutletTemp = evaporatorOutletNTC.temperature();
    condenserInletTemp = condenserInletNTC.temperature();
    condenserOutletTemp = condenserOutletNTC.temperature();
    ambientTemp = ambientNTC.temperature();
    coolingPower = (evaporatorInletA10Temp - evaporatorOutletTemp) * SPECIFIC_HEAT * flowRate / 60000; // Watts
}

void CoolerSystem::updateFaults() {
    // check faults and set systemFault flags
    check(systemPressure < OVERPRESSURE_THRESHOLD_KPA, SystemFault::SYSTEM_OVER_PRESSURE);
    check(coolantLevel, SystemFault::LOW_COOLANT);
    check(compressorFaultCode == CompressorFaultCode::OK, SystemFault::COMPRESSOR_FAULT);
    check(!voltageMonitor.overVoltage(), SystemFault::SYSTEM_OVERVOLT);
    check(!voltageMonitor.underVoltage(), SystemFault::SYSTEM_UNDERVOLT);
    compressorFaultCode = compressorFault.getCode();

    // check flow rate after chiller pump startup period is over and verify it's sufficient
    if (chillerPumpPWM.value()) {
        int32_t pumpRunTime = millis() - pumpStartTime;
        bool flowError = flowRate < FLOW_RATE_MIN_THRESHOLD && pumpRunTime >= FLOW_RATE_STARTUP_TIME;
        check(!flowError, SystemFault::FLOW_RATE_LOW);
    }
}

void CoolerSystem::updateState()
{
    switch (systemStatus) {
        case CoolerSystemStatus::STARTUP:
            if (sampleCounter < STARTUP_STABILIZATION_SAMPLES) {
                return;
            }

            // align clock and timers to when sample stabilization is complete and we're ready
            // to process inputs for the first time
            ClockTime::setEpoch();
            updateStateTimer.reset();
            displayInfoTimer.reset();

            systemStatus = CoolerSystemStatus::REQUIRES_RESET;
            // fall through to full state update logic, so the first update is aligned
            // with the epoch

        default:
            // this will fire on the first call, and thereafter on every interval period
            if (!updateStateTimer.check()) {
                return;
            }

            updateCoolerData();
            updateFaults();

            // handle system flushing
            if (systemStatus != CoolerSystemStatus::FLUSH && shouldFlush) {
                // start flushing
                systemStatus = CoolerSystemStatus::FLUSH;
                pumpStartTime = millis();
                return;
            }
            if (systemStatus == CoolerSystemStatus::FLUSH && shouldFlush && millis() < pumpStartTime + FLUSH_TIMEOUT_MS) {
                // continue flushing, ignore faults and other inputs
                return;
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
                return;
            }

            if (systemStatus == CoolerSystemStatus::REQUIRES_RESET) {
                // don't exit this state unless switch is returned to RESET position
                return;
            }

            switch (switchPosition) {
                case CoolerSwitchPosition::PRECHILL:
                    systemStatus = CoolerSystemStatus::PRECHILL;
                    return;

                case CoolerSwitchPosition::PUMP_LOW:
                    systemStatus = CoolerSystemStatus::PUMP_LOW;
                    return;

                case CoolerSwitchPosition::PUMP_MEDIUM:
                    systemStatus = CoolerSystemStatus::PUMP_MEDIUM;
                    return;

                case CoolerSwitchPosition::PUMP_HIGH:
                    systemStatus = CoolerSystemStatus::PUMP_HIGH;
                    return;

                default:
                    // do nothing
                    return;
            }
    }
}

void CoolerSystem::updateOutputs()
{
    switch (systemStatus) {
        case CoolerSystemStatus::STARTUP:
            return;

        default:
            runChillerPump();
            runCompressor();
            runCoolshirtPump();
            compressorPID.Compute();
            return;
    }
}

void CoolerSystem::displayInfo()
{
#ifndef DEBUGLOG_DISABLE_LOG
    if (systemStatus == CoolerSystemStatus::STARTUP || !displayInfoTimer.check()) {
        return;
    }

    char str[2048];
    StringFormatCSV format(str, sizeof(str), 0);

    format.formatLiteral("----------------- Cooler Statistics -------------------\n");
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
    format.formatFloat3DP(evaporatorInletA10Temp);
    format.formatLiteral(" °C ( ");
    format.formatUnsignedInt(evaporatorInletA10.adc());
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
    format.formatFloat3DP(flowRate / 1000.0);
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

    format.formatLiteral("  Chiller        :               ");
    systemEnableOutput.value() ? format.formatLiteral("ON\n") : format.formatLiteral("OFF\n");

    format.formatLiteral("  Chiller Pump:                  ");
    chillerPumpPWM.value() ? format.formatLiteral("ON\n") : format.formatLiteral("OFF\n");

    format.formatLiteral("  Coolshirt Pump:                ");
    coolshirtPWM.value() ? format.formatLiteral("ON\n") : format.formatLiteral("OFF\n");

    format.formatLiteral("  Compressor Speed:              ");
    format.formatUnsignedInt(compressorSpeed * 100);
    format.formatLiteral(" %\n");

    format.formatLiteral("  Undertemp Cutoff:              ");
    undertempCutoff ? format.formatLiteral("CUTOFF\n") : format.formatLiteral("OK\n");

    format.formatLiteral("  Cooling Power:                 ");
    format.formatFloat3DP(coolingPower);
    format.formatLiteral(" W\n");

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
    updateState();
    if (systemStatus != prevStatus) {
        LOG_INFO("System status changed from", CoolerSystemStatusToString(prevStatus), "to", CoolerSystemStatusToString(systemStatus));
    }

    if (systemStatus != CoolerSystemStatus::STARTUP) {
        uint32_t sampleTime = ClockTime::millisSinceEpoch();
#if NTC_DEBUG
        ntcLogger.ensureSetup("time,inlet,inletA10,current");
        ntcLogger.logSamples(sampleTime, evaporatorInletNTC.latest(), evaporatorInletA10.latest(), currentSensor.latest(), 0);
#endif
#if FLOW_DEBUG
        if (flowSensor.lastPulseIndex() != lastLoggedFlowPulse) {
            //LOG_INFO("flow pulse!", flowSensor.lastPulseDuration());
            ntcLogger.ensureSetup("time,index,duration");
            ntcLogger.logSamples(sampleTime, flowSensor.lastPulseIndex(), flowSensor.lastPulseDuration(), 0, 0);
            lastLoggedFlowPulse = flowSensor.lastPulseIndex();
        }
#endif
    }

    updateOutputs();
    displayInfo();
};

void CoolerSystem::getSystemData(CoolerSystemData &data) {
    data.evaporatorInletTemp = evaporatorInletTemp;
    data.evaporatorOutletTemp = evaporatorOutletTemp;
    data.condenserInletTemp = condenserInletTemp;
    data.condenserOutletTemp = condenserOutletTemp;
    data.ambientTemp = ambientTemp;
    data.compressorSpeed = compressorSpeed;
    data.coolantLevel = coolantLevel;
    data.fault = _systemFault;
    data.flowRate = flowRate;
    data.systemPressure = systemPressure;
    data.compressorCurrent = compressorCurrent;
};

unsigned long CoolerSystem::lastFlowPulseMicros() {
    return flowSensor.lastPulseMicros();
}

void CoolerSystem::setCompressorSpeedPercent(uint32_t percent) {
    compressorSpeed = (double) percent / 100;
    LOG_INFO("Setting compressor speed to", percent, "%");
}

void CoolerSystem::toggleFlush() {
    shouldFlush = systemStatus != CoolerSystemStatus::FLUSH;
}

byte CoolerSystem::systemFault() {
    return _systemFault;
}

uint8_t clampAndScale(float val, int minVal, int scale) {
    return max(255, (uint8_t)(max(minVal, val)) - minVal) / scale;
}

void CoolerSystem::getCANMessage(CAN_message_t &msg)
{
    msg.id = CANID_COOLER_SYSTEM;
    msg.len = 8;
    // msg.flags = {};

    // byte | purpose
    // ------------------------------
    // 0    | chiller inlet temp (celcius) uint8_t byte = (T + 15) / 0.25
    // 1    | chiller outlet temp (celcius) uint8_t byte = (T + 15) / 0.25
    // 2    | flow rate (cL/min [100mL/min]) uint8_t
    // 3    | system pressure kPa scaling factor 0.25 uint8_t
    // 4    | compressor value 0-254 uint8_t
    // 5    | coolshirt value 0-254 uint8_t
    // 6    | flags bitmap
    //      |   [7]: system enable
    //      |   [6]: chiller pump active
    //      |   [5]: system reset required
    //      |   [4]: coolant level OK
    //      |   [3]: under-temp compressor cut-off
    //      |   [2,1,0]: system status
    // 7    | system faults
    // 8    | compressor fault

    msg.buf[0] = clampAndScale(evaporatorInletTemp, -5, 0.2);
    msg.buf[1] = evaporatorOutletTemp > -5 ? uint8_t((evaporatorOutletTemp + 5) / 0.2): 0xFF;
    msg.buf[0] = condenserInletTemp > 10 ? uint8_t((condenserInletTemp - 10) / 0.35) : 0xFF;
    msg.buf[1] = condenserOutletTemp > 10 ? uint8_t((condenserOutletTemp - 10) / 0.35): 0xFF;
    msg.buf[1] = ambientTemp > 10 ? uint8_t((ambientTemp - 10) / 0.2): 0xFF;
    msg.buf[2] = (uint8_t)(flowRate / 15);
    msg.buf[3] = systemPressure / 4;
    msg.buf[4] = compressorSpeed * 255;
    msg.buf[5] = (uint8_t)(compressorCurrent / 40 * 255);
    msg.buf[6] = coolshirtPWM.value() >> 8;
    msg.buf[7] = 0;
    if (systemEnableOutput.value()) msg.buf[7] |= 0x80;
    if (chillerPumpPWM.value() >> 8) msg.buf[7] |= 0x40;
    if (_systemFault) msg.buf[7] |= 0x20;
    if (coolantLevel) msg.buf[7] |= 0x10;
    if (undertempCutoff) msg.buf[7] |= 0x08;
    msg.buf[7] |= (0x07 & (uint8_t)systemStatus);
    //msg.buf[8] = (uint8_t)_systemFault;
    //msg.buf[9] = (uint8_t)compressorFault.getCode();
};

void CoolerSystem::getLogMessage(StringFormatCSV& format)
{
    format.formatFloat3DP(ClockTime::secSinceEpoch());
    format.formatFloat3DP(evaporatorInletTemp);
    format.formatFloat3DP(evaporatorInletA10Temp);
    format.formatFloat3DP(evaporatorOutletTemp);
    format.formatFloat3DP(condenserInletTemp);
    format.formatFloat3DP(condenserOutletTemp);
    format.formatFloat3DP(ambientTemp);
    format.formatFloat3DP(flowRate / 1000.0);
    format.formatUnsignedInt(systemPressure);
    format.formatBool(coolantLevel);
    format.formatFloat3DP((float) voltageMonitor.get12vMilliVolts() / 1000);
    format.formatFloat3DP((float) voltageMonitor.get5vMilliVolts() / 1000);
    format.formatFloat3DP((float) voltageMonitor.get3v3MilliVolts() / 1000);
    format.formatFloat3DP((float) voltageMonitor.getp3v3MilliVolts() / 1000);
    format.formatInt(coolingPower);
    format.formatInt((int32_t) switchADC.position());
    format.formatUnsignedInt(switchADC.adc());
    format.formatInt((int32_t) systemStatus);
    format.formatBool(systemEnableOutput.value());
    format.formatBool(chillerPumpPWM.value());
    format.formatBool(coolshirtPWM.value());
    format.formatFloat3DP(compressorSpeed);
    format.formatBool(undertempCutoff);
    format.formatBinary(_systemFault);
    format.formatUnsignedInt((uint32_t) compressorFaultCode);
};
