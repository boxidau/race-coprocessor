#include "coolersystem.h"
#include "clocktime.h"

void printFaultLine(SystemFault f, byte systemFault) {
    const char* faultName = SystemFaultToString(f);
    uint prefLength = snprintf(nullptr, 0, "  %s ( %d ):", faultName, (uint8_t)f);
    Serial.printf("  %s ( %d ):%s%s\n",
        faultName,
        (uint8_t)f,
        String("             ").substring(prefLength-20).c_str(),
        (systemFault & (byte)f) ? "ALARM" : "OK"
    );
};

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
    analogWrite(compressorSpeedPin, 1 * COMPRESSOR_SPEED_RATIO_TO_ANALOG); // 9V = 100%
    compressorPID.SetOutputLimits(COMPRESSOR_MIN_SPEED_RATIO, COMPRESSOR_MAX_SPEED_RATIO);
    compressorPID.SetSampleTime(POLL_TIMER_MS);
    compressorPID.SetMode(MANUAL);

    voltageMonitor.setup();
};

void CoolerSystem::_pollSystemStatus()
{
    CoolerSwitchPosition _newSwitchPosition = switchADC.position();
    if (_newSwitchPosition == CoolerSwitchPosition::RESET) {
        _systemFault = 0;
    }

    CoolerSystemStatus _newSystemStatus = systemStatus;
    if (_systemFault > 0) {
        _newSystemStatus = CoolerSystemStatus::REQUIRES_RESET;
    } else if ((_newSystemStatus == CoolerSystemStatus::REQUIRES_RESET && _newSwitchPosition != CoolerSwitchPosition::RESET) ||
        _newSwitchPosition == CoolerSwitchPosition::UNKNOWN) {
        return;
    } else {
        _newSystemStatus = CoolerSwitchPositionToStatus(_newSwitchPosition);
    }

    if (_newSystemStatus != systemStatus) {
        LOG_INFO("System status changed", CoolerSystemStatusToString(systemStatus), "to", CoolerSystemStatusToString(_newSystemStatus));
        systemStatus = _newSystemStatus;
    }
}

void CoolerSystem::_pollCoolantLevel()
{
    if (coolantLevelBounce.update()) {
        coolantLevel = coolantLevelBounce.read();
    }
}

void CoolerSystem::runChillerPump()
{
    // doesn't use system status since we want to run the pump in specific fault modes
    if (switchADC.position() == CoolerSwitchPosition::RESET) {
        return chillerPumpPWM.set(0);
    }

    if (
        (_systemFault & uint8_t(SystemFault::LOW_COOLANT))
        || (_systemFault & uint8_t(SystemFault::SYSTEM_STARTUP))
    ) {
        return chillerPumpPWM.set(0);
    }

    if (chillerPumpPWM.value() == 0) {
        pumpStartTime = millis();
        chillerPumpPWM.setPercent(80);
    }
    uint32_t pumpRunTime = millis() - pumpStartTime;
    if (chillerPumpPWM.percent() == 80 && pumpRunTime > 1000) {
        chillerPumpPWM.setPercent(40);
    }
    // we haven't seen enough flow after some amount of time
    // maybe the pump is broken
    bool flowError = (flowRate <= FLOW_RATE_MIN_THRESHOLD && pumpRunTime >= FLOW_RATE_STARTUP_TIME);
    if (flowError) LOG_ERROR("Pump started", pumpRunTime, "ms ago, inadequate pump flow (", flowRate, "), panic!");
    check(!flowError, SystemFault::FLOW_RATE_LOW);
};

void CoolerSystem::runCompressor()
{
    // undertemp check
    // this isn't a panic condition
    // but we are going to shut down the compressor until temp
    // goes back above a safe value
    float cutoffTemp = 0, restartTemp = 0;
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
        default:
            cutoffTemp = COMPRESSOR_UNDER_TEMP_CUTOFF_HIGH;
            restartTemp = COMPRESSOR_RESTART_TEMP_HIGH;
            break;
    }
    compressorTempTarget = (cutoffTemp + restartTemp) / 2;

    if (evaporatorInletTemp <= cutoffTemp && !undertempCutoff) {
        undertempCutoff = true;
    } else if (evaporatorInletTemp >= restartTemp && undertempCutoff) {
        undertempCutoff = false;
    }

    if (systemStatus == CoolerSystemStatus::RESET || systemStatus == CoolerSystemStatus::REQUIRES_RESET || undertempCutoff) {
        systemEnableOutput.setBoolean(false);
        analogWrite(compressorSpeedPin, 0);
        compressorPID.SetMode(MANUAL);
    } else {
        systemEnableOutput.setBoolean(true);
        compressorPID.SetMode(AUTOMATIC);
        analogWrite(compressorSpeedPin, 0.75 * COMPRESSOR_SPEED_RATIO_TO_ANALOG); // 9V = 100%, 4.5V = 50%.
        // LOG_DEBUG("Compressor input temp:", compressorInputTemp, "target temp:", compressorTempTarget, "output value", compressorSpeed);
    }
}

void CoolerSystem::runCoolshirtPump()
{
    switch(systemStatus) {
        case CoolerSystemStatus::PUMP_LOW:
            //return coolshirtPWM.setPercent(33);
        case CoolerSystemStatus::PUMP_MEDIUM:
            //return coolshirtPWM.setPercent(66);
        case CoolerSystemStatus::PUMP_HIGH:
            return coolshirtPWM.setPercent(70);
        default:
            return coolshirtPWM.setPercent(0);
    }
}

void CoolerSystem::check(bool checkResult, SystemFault fault)
{
    if (!checkResult) {
        // fault code raised, assertion is false
        _systemFault |= (byte)fault;
    }
}

bool CoolerSystem::hasStarted() {
    return startupCounter == 150;
}

static bool highNTC = false;

void CoolerSystem::loop()
{
    loopTimer.start();
    if (firstLoop) {
        // reset the timers so they're in sync with when the first sample was taken
        msTick.reset();
        //pollTimer.reset();
        //displayInfoTimer.reset();
        firstLoop = false;
        startTimeIndex = micros();
    }

    if (!msTick.check()) {
        return;
    }
    if (startupCounter < 150) {
        startupCounter++;
        if (startupCounter == 150) {
            ClockTime::setEpoch();
        }
    }
    pollCounter++;
    displayInfoCounter++;

    // sample evap inlet after outlet, for reduced noise
    uint32_t sampleTime = ClockTime::millisSinceEpoch();
    evaporatorOutletNTC.loop();
    evaporatorInletNTC.loop();
    condenserInletNTC.loop();
    condenserOutletNTC.loop();
    ambientNTC.loop();

    pressureSensor.loop();
    currentSensor.loop();
    compressorFault.loop();
    _pollCoolantLevel();
    switchADC.loop();
    voltageMonitor.loop();

    if (NTC_DEBUG) {
        ntcLogger.ensureSetup();
        ntcLogger.logSamples(sampleTime, evaporatorInletNTC.latest(), evaporatorInletNTC.adc(), evaporatorOutletNTC.latest(), evaporatorOutletNTC.adc());
    }

    if (startupCounter < 150) {
        // allow samples to stabilize before we run loops
        return;
    }

    if (evaporatorInletNTC.latest() > 65500 || evaporatorOutletNTC.latest() > 65500) {
        highNTC = true;
    }

    if (pollCounter > POLL_TIMER_MS) {
        pollCounter = 0;
        flowRate = flowSensor.flowRate();
        systemPressure = pressureSensor.calibratedValue();
        compressorCurrent = currentSensor.calibratedValue();
        evaporatorInletTemp = evaporatorInletNTC.temperature();
        evaporatorOutletTemp = evaporatorOutletNTC.temperature();
        condenserInletTemp = condenserInletNTC.temperature();
        condenserOutletTemp = condenserOutletNTC.temperature();
        ambientTemp = ambientNTC.temperature();
        coolingPower = (evaporatorInletTemp - evaporatorOutletTemp) * SPECIFIC_HEAT * flowRate / 60000; // Watts
        compressorFaultCode = compressorFault.getCode();
        _pollSystemStatus();
        check(systemPressure < OVERPRESSURE_THRESHOLD_KPA, SystemFault::SYSTEM_OVER_PRESSURE);
        check(coolantLevel, SystemFault::LOW_COOLANT);
        check(!voltageMonitor.overVoltage(), SystemFault::SYSTEM_OVERVOLT);
        check(!voltageMonitor.underVoltage(), SystemFault::SYSTEM_UNDERVOLT);

        // begin actions
        runChillerPump();
        runCompressor();
        runCoolshirtPump();
        compressorPID.Compute();
    }

    if (displayInfoCounter > DISPLAY_INFO_MS) {
        displayInfoCounter = 0;

        if (!SHOW_INFO) {
            return;
        }

        Serial.println("----------------- Cooler Statistics -------------------");
        Serial.println("Voltages ----------------------------------------------");
        Serial.printf("  System 12V:                    %.2f V\n", voltageMonitor.get12vMilliVolts() / 1000.0);
        Serial.printf("  System 5V:                     %.2f V\n", voltageMonitor.get5vMilliVolts() / 1000.0);
        Serial.printf("  System 3.3V:                   %.2f V\n", voltageMonitor.get3v3MilliVolts() / 1000.0);
        Serial.printf("  System P3.3V:                  %.2f V\n", voltageMonitor.getp3v3MilliVolts() / 1000.0);
        Serial.println("Inputs ------------------------------------------------");
        Serial.printf("  Evaporator Inlet:              %.2f °C ( %d )\n", evaporatorInletTemp, (int) evaporatorInletNTC.adc());
        Serial.printf("  Evaporator Outlet:             %.2f °C ( %d )\n", evaporatorOutletTemp, (int) evaporatorOutletNTC.adc());
        Serial.printf("  Condenser Inlet:               %.2f °C ( %d )\n", condenserInletTemp, (int) condenserInletNTC.adc());
        Serial.printf("  Condenser Outlet:              %.2f °C ( %d )\n", condenserOutletTemp, (int) condenserOutletNTC.adc());
        Serial.printf("  Ambient Temp:                  %.2f °C ( %d )\n", ambientTemp, (int) ambientNTC.adc());
        Serial.printf("  Cooling Power:                 %.2f W\n", coolingPower);
        Serial.printf("  System Pressure:               %d kPa ( %d )\n", systemPressure, pressureSensor.adc());
        Serial.printf("  Coolant Level:                 %s\n", coolantLevel ? "OK" : "LOW");
        Serial.printf("  Compressor Current:            %.2f A ( %d )\n", compressorCurrent, currentSensor.adc());
        Serial.printf("  Compressor Status:             %s (%d ms ago)\n", CompressorFaultToString(compressorFaultCode), compressorFault.durationSinceFaultRecorded());
        Serial.printf("  Flow Rate:                     %d mL/min\n", flowRate);
        Serial.printf("  Coolant Level:                 %s\n", coolantLevel ? "OK" : "LOW");
        Serial.printf("  Switch Position:               %s ( %d )\n", CoolerSwitchPositionToString(switchADC.position()), switchADC.adc());
        Serial.println("Outputs -----------------------------------------------");
        Serial.printf("  System Status:                 %s\n", CoolerSystemStatusToString(systemStatus));
        Serial.printf("  System Enabled:                %s\n", systemEnableOutput.value() ? "ON" : "OFF");
        Serial.printf("  Compressor Speed:              %d %%\n", compressorSpeed * 100);
        Serial.printf("  Chiller Pump:                  %s\n", chillerPumpPWM.value() ? "ON" : "OFF");
        Serial.printf("  Coolshirt PWM:                 %d %%\n", coolshirtPWM.percent());
        Serial.printf("  Undertemp Cutoff:              %s\n", undertempCutoff ? "CUTOFF" : "OK");
        Serial.println("Faults ------------------------------------------------");
        printFaultLine(SystemFault::LOW_COOLANT, _systemFault);
        printFaultLine(SystemFault::FLOW_RATE_LOW, _systemFault);
        printFaultLine(SystemFault::SYSTEM_OVER_PRESSURE, _systemFault);
        printFaultLine(SystemFault::SYSTEM_UNDERVOLT, _systemFault);
        printFaultLine(SystemFault::SYSTEM_OVERVOLT, _systemFault);
        printFaultLine(SystemFault::SYSTEM_STARTUP, _systemFault);
        printFaultLine(SystemFault::COMPRESSOR_FAULT, _systemFault);
        Serial.println("-------------------------------------------------------");
    }
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
    // format.formatUnsignedInt(evaporatorInletNTC.adc());
    // format.formatUnsignedInt(evaporatorOutletNTC.adc());
    // format.formatUnsignedInt(condenserInletNTC.adc());
    // format.formatUnsignedInt(condenserOutletNTC.adc());
    // format.formatUnsignedInt(ambientNTC.adc());
    format.formatFloat3DP(evaporatorInletTemp);
    format.formatFloat3DP(evaporatorOutletTemp);
    format.formatFloat3DP(condenserInletTemp);
    format.formatFloat3DP(condenserOutletTemp);
    format.formatFloat3DP(ambientTemp);
    format.formatBool(highNTC); highNTC = false;
    format.formatUnsignedInt(flowRate);
    format.formatUnsignedInt(systemPressure);
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
};
