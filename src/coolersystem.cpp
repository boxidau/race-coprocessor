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
        PRESSURE_SENSOR_CALIBRAION_HIGH_ADC,
        PRESSURE_SENSOR_CALIBRATION_HIGH_KPA,
        true
    );

    pinMode(compressorSpeedPin, OUTPUT);
    compressorPID = PID(
        &evaporatorOutletTemp, &compressorSpeed, &compressorTempTarget,
        Kp, Ki, Kd, P_ON_M, REVERSE
    );
    compressorPID.SetOutputLimits(COMPRESSOR_MIN_SPEED_RATIO, COMPRESSOR_MAX_SPEED_RATIO);
    compressorPID.setSampleTime(1);

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
        compressorPID.setMode(MANUAL);
    } else {
        systemEnableOutput.setBoolean(true);
        compressorPID.setMode(AUTOMATIC);
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
    if (msTick.check()) {
        evaporatorInletNTC.loop();
        evaporatorOutletNTC.loop();
        condenserInletNTC.loop();
        condenserOutletNTC.loop();
        ambientNTC.loop();

        pressureSensor.loop();
        _pollCoolantLevel();
        switchADC.loop();
        voltageMonitor.loop();
        compressorPID.Compute();
    }
    //flowSensor.loop();

    if (pollTimer.check()) {
        flowRate = flowSensor.flowRate();
        systemPressure = pressureSensor.calibratedValue() * voltageMonitor.get5vMilliVolts() / 5000;
        evaporatorInletTemp = evaporatorInletNTC.temperature();
        evaporatorOutletTemp = evaporatorOutletNTC.temperature();
        condenserInletTemp = condenserInletNTC.temperature();
        condenserOutletTemp = condenserOutletNTC.temperature();
        ambientTemp = ambientNTC.temperature();
        _pollSystemStatus();
        check(systemPressure < OVERPRESSURE_THRESHOLD_KPA, SystemFault::SYSTEM_OVER_PRESSURE);
        check(coolantLevel, SystemFault::LOW_COOLANT);
        check(!voltageMonitor.overVoltage(), SystemFault::SYSTEM_OVERVOLT);
        check(!voltageMonitor.underVoltage(), SystemFault::SYSTEM_UNDERVOLT);

        // begin actions
        runChillerPump();
        runCompressor();
        runCoolshirtPump();
    }

    if (NTC_DEBUG && displayInfoTimer.check()) {
        uint16_t min = ambientNTC.min(), max = ambientNTC.max();
        Serial.printf(
            "Ambient Temp: avg %5d  (%.2f)    stdev %5d  (%.2f)    range %5d  (%.2f)\n",
            ambientNTC.adc(),
            ambientNTC.temperature(),
            ambientNTC.stdev(),
            ambientNTC.temperatureStdev(),
            max - min,
            ambientNTC.temperatureFor(max) - ambientNTC.temperatureFor(min)
        );
        return;
    }

    if (displayInfoTimer.check()) {
        Serial.println("----------------- Cooler Statistics -------------------");
        Serial.println("Voltages ----------------------------------------------");
        Serial.printf("  System 12V:                    %.2f V\n", voltageMonitor.get12vMilliVolts() / 1000.0);
        Serial.printf("  System 5V:                     %.2f V\n", voltageMonitor.get5vMilliVolts() / 1000.0);
        Serial.printf("  System 3.3V:                   %.2f V\n", voltageMonitor.get3v3MilliVolts() / 1000.0);
        Serial.println("Inputs ------------------------------------------------");
        Serial.printf("  Evaporator Inlet:              %.2f °C ( %d )\n", evaporatorInletTemp, evaporatorInletNTC.adc());
        Serial.printf("  Evaporator Outlet:             %.2f °C ( %d )\n", evaporatorOutletTemp, evaporatorOutletNTC.adc());
        Serial.printf("  Condenser Inlet:               %.2f °C ( %d )\n", condenserInletTemp, condenserInletNTC.adc());
        Serial.printf("  Condenser Outlet:              %.2f °C ( %d )\n", condenserOutletTemp, condenserOutletNTC.adc());
        Serial.printf("  Ambient Temp:                  %.2f °C ( %d )\n", ambientTemp, ambientNTC.adc());
        Serial.printf("  System Pressure:               %d kPa ( %d )\n", systemPressure, pressureSensor.adc());
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
};

byte CoolerSystem::systemFault() {
    return _systemFault;
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

    msg.buf[0] = evaporatorInletTemp > -15 ? uint8_t((evaporatorInletTemp + 15) / 0.25) : 0xFF;
    msg.buf[1] = evaporatorOutletTemp > -15 ? uint8_t((evaporatorOutletTemp + 15) / 0.25): 0xFF;
    msg.buf[2] = (uint8_t)(flowRate / 15);
    msg.buf[3] = systemPressure / 4;
    msg.buf[4] = compressorSpeed * 256;
    msg.buf[5] = coolshirtPWM.value() >> 8;
    msg.buf[6] = 0;
    if (systemEnableOutput.value()) msg.buf[6] |= 0x80;
    if (chillerPumpPWM.value() >> 8) msg.buf[6] |= 0x40;
    if (_systemFault) msg.buf[6] |= 0x20;
    if (coolantLevel) msg.buf[6] |= 0x10;
    if (undertempCutoff) msg.buf[6] |= 0x08;
    msg.buf[6] |= (0x07 & (uint8_t)systemStatus);
    msg.buf[7] = (uint8_t)_systemFault;
};
