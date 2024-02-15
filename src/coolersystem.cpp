#include "coolersystem.h"

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
    evaporatorDifferentialNTC.setup();
    condenserInletNTC.setup();
    condenserOutletNTC.setup();
    ambientNTC.setup();
    if (NTC_DEBUG) {
        ntcLogger.setup();
    }

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
    }

    // if switch is UNKNOWN it's likely in between positions, don't change system state
    if (_newSwitchPosition != CoolerSwitchPosition::UNKNOWN) {
        _newSystemStatus = CoolerSwitchPositionToStatus(_newSwitchPosition);
    }

    if (_newSystemStatus != systemStatus) {
        LOG_INFO("Switch position changed", CoolerSystemStatusToString(systemStatus), "to", CoolerSystemStatusToString(_newSystemStatus));
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
        chillerPumpPWM.setPercent(66);
    }
    uint32_t pumpRunTime = millis() - pumpStartTime;
    // we haven't seen enough flow after some amount of time
    // maybe the pump is broken
    bool flowError = (flowRate <= FLOW_RATE_MIN_THRESHOLD && pumpRunTime >= FLOW_RATE_MIN_TIME_MS_THRESHOLD);
    if (flowError) LOG_ERROR("Pump started", pumpRunTime, "ms ago, inadequate pump flow (", flowRate, "), panic!");
    check(!flowError, SystemFault::FLOW_RATE_LOW);
};


void CoolerSystem::runCompressor()
{
    // undertemp check
    // this isn't a panic condition
    // but we are going to shut down the compressor until temp
    // goes back above a safe value
    if (evaporatorInletTemp <= COMPRESSOR_UNDER_TEMP_CUTOFF && !undertempCutoff) {
        undertempCutoff = true;
        LOG_WARN("Compressor stopped due to under temp cut-off, current temp", evaporatorInletTemp);
    }
    if (evaporatorInletTemp >= COMPRESSOR_RESUME_PID_CONTROL_TEMP && undertempCutoff) {
        undertempCutoff = false;
        LOG_INFO("Resume compressor automated control");
    }

    if (systemStatus < CoolerSystemStatus::PRECHILL || undertempCutoff) {
        systemEnableOutput.setBoolean(false);
        analogWrite(compressorSpeedPin, 0);
        compressorPID.SetMode(MANUAL);
    } else {
        systemEnableOutput.setBoolean(true);
        // compressorPID.SetMode(AUTOMATIC);
        analogWrite(compressorSpeedPin, 1 * COMPRESSOR_SPEED_RATIO_TO_ANALOG); // 9V = 100%, 4.5V = 50%.
        // LOG_DEBUG("Compressor input temp:", compressorInputTemp, "target temp:", compressorTempTarget, "output value", compressorSpeed);
    }
}

void CoolerSystem::runCoolshirtPump()
{
    switch(systemStatus) {
        case CoolerSystemStatus::PUMP_LOW:
            return coolshirtPWM.setPercent(33);
        case CoolerSystemStatus::PUMP_MEDIUM:
            return coolshirtPWM.setPercent(66);
        case CoolerSystemStatus::PUMP_HIGH:
            return coolshirtPWM.setPercent(100);
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

void CoolerSystem::loop()
{
    if (firstLoop) {
        // reset the timers so they're in sync with when the first sample was taken
        pollTimer.reset();
        displayInfoTimer.reset();
        firstLoop = false;
        startTimeIndex = micros();
    }

    if (!msTick.check()) {
        return;
    }

    unsigned long ntcSampleDuration = micros();
    unsigned long loopStart = ntcSampleDuration;
    evaporatorInletNTC.loop();
    evaporatorOutletNTC.loop();
    evaporatorDifferentialNTC.loop();
    condenserInletNTC.loop();
    condenserOutletNTC.loop();
    ambientNTC.loop();
    ntcSampleDuration = micros() - ntcSampleDuration;

    pressureSensor.loop();
    currentSensor.loop();
    compressorFault.loop();
    _pollCoolantLevel();
    switchADC.loop();
    voltageMonitor.loop();

    if (NTC_DEBUG) {
        ntcLogger.logSamples(evaporatorInletNTC.latest(), evaporatorOutletNTC.latest(), evaporatorDifferentialNTC.latest(), condenserInletNTC.latest(), condenserOutletNTC.latest(), ambientNTC.latest());
    }
    //flowSensor.loop();

    if (pollTimer.check()) {
        flowRate = flowSensor.flowRate();
        systemPressure = pressureSensor.calibratedValue();
        compressorCurrent = currentSensor.calibratedValue();
        evaporatorInletTemp = evaporatorInletNTC.temperature();
        evaporatorOutletTemp = evaporatorOutletNTC.temperature();
        evaporatorDifferentialTemp = evaporatorInletTemp + evaporatorInletNTC.temperatureFor(evaporatorInletNTC.adc() + evaporatorDifferentialNTC.adc());
        condenserInletTemp = condenserInletNTC.temperature();
        condenserOutletTemp = condenserOutletNTC.temperature();
        ambientTemp = ambientNTC.temperature();
        coolingPower = (evaporatorInletTemp - evaporatorOutletTemp) * SPECIFIC_HEAT * (double) flowRate / 60000; // Watts
        compressorFaultCode = compressorFault.getCode();
        _pollSystemStatus();
        check(systemPressure < OVERPRESSURE_THRESHOLD_KPA, SystemFault::SYSTEM_OVER_PRESSURE);
        check(coolantLevel, SystemFault::LOW_COOLANT);
        check(!voltageMonitor.overVoltage(), SystemFault::SYSTEM_OVERVOLT);
        check(!voltageMonitor.underVoltage(), SystemFault::SYSTEM_UNDERVOLT);

        // begin actions
        compressorPID.Compute();
        runChillerPump();
        runCompressor();
        runCoolshirtPump();
    }

    if (displayInfoTimer.check()) {
        if (NTC_DEBUG) {
            Serial.printf(
                "[%.3f s] Ambient Temp: avg %5d (%.3f C)    duration %u us\n",
                (double) ntcLogger.msSinceStarted() / 1000,
                ambientNTC.adc(),
                ambientNTC.temperature(),
                ntcSampleDuration
            );
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

uint8_t clampAndScale(double val, int minVal, int scale) {
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

void CoolerSystem::getLogMessage(char* message)
{
    sprintf(message, "%.3f,%.3f,%.3f,%u,%u,%.3f,%u,%s,%s,%s,%s,%u,%s",
        (double)(micros() - startTimeIndex) / 1e6,
        evaporatorInletTemp,
        evaporatorOutletTemp,
        evaporatorInletNTC.adc(),
        evaporatorOutletNTC.adc(),
        ambientTemp,
        flowRate,
        CoolerSwitchPositionToString(switchADC.position()),
        CoolerSystemStatusToString(systemStatus),
        systemEnableOutput.value() ? "ON" : "OFF",
        chillerPumpPWM.value() ? "ON" : "OFF",
        coolshirtPWM.percent(),
        undertempCutoff ? "CUTOFF" : "OK"
    );
    LOG_INFO(message);
};
