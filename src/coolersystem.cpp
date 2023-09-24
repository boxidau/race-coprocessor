#include "coolersystem.h"

void CoolerSystem::setup()
{
    // setup pin modes and initial states
    pinMode(compressorPin, OUTPUT);
    digitalWriteFast(compressorPin, HIGH);
    pinMode(chillerPumpPin, OUTPUT);
    digitalWriteFast(chillerPumpPin, LOW);
    pinMode(coolshirtPumpPin, OUTPUT);
    digitalWriteFast(coolshirtPumpPin, LOW);
    pinMode(systemEnablePin, OUTPUT);
    digitalWriteFast(systemEnablePin, LOW);
    pinMode(flowPulsePin, OUTPUT);
    analogWriteFrequency(compressorPin, 18000);
    analogWriteFrequency(coolshirtPumpPin, 18000);
    analogWriteFrequency(systemEnablePin, 18000);

    pinMode(switchPin, INPUT);
    pinMode(coolantLevelPin, INPUT);
    pinMode(flowRatePin, INPUT);
    pinMode(pressureSensorPin, INPUT);

    compressorPID.SetOutputLimits(0, ADC_MAX);
    compressorPID.SetMode(AUTOMATIC);
};

CoolerSystemStatus CoolerSystem::_getSwitchPosition()
{
    static CoolerSystemStatus switchPosition = CoolerSystemStatus::RESET;
    const uint16_t switchADCValue = analogRead(switchPin);
    if (switchADCValue < 760) switchPosition = CoolerSystemStatus::RESET;
    if (switchADCValue < 1136) switchPosition = CoolerSystemStatus::PRECHILL;
    if (switchADCValue < 1360) switchPosition = CoolerSystemStatus::PUMP_LOW;
    if (switchADCValue < 1520) switchPosition = CoolerSystemStatus::PUMP_MEDIUM;
    if (switchADCValue < 1640) switchPosition = CoolerSystemStatus::PUMP_HIGH;
    return switchPosition;
}

void CoolerSystem::_pollSystemStatus()
{
    CoolerSystemStatus _newSwitchPosition = _getSwitchPosition();

    if (_systemFault > 0 && _newSwitchPosition != CoolerSystemStatus::RESET) {
        _newSwitchPosition = CoolerSystemStatus::REQUIRES_RESET;
    }
    if (_newSwitchPosition == CoolerSystemStatus::RESET) {
        _systemFault = 0;
    }
    if (_newSwitchPosition != systemStatus) {
        LOG_INFO("Switch position changed", CoolerSystemStatusToString(systemStatus), "to", CoolerSystemStatusToString(_newSwitchPosition));
        systemStatus = _newSwitchPosition;
    }
}

void CoolerSystem::_pollSystemPressure()
{
    systemPressure = map(
        constrain(
            analogRead(pressureSensorPin),
            PRESSURE_SENSOR_CALIBRATION_LOW_ADC,
            PRESSURE_SENSOR_CALIBRAION_HIGH_ADC
        ),
        PRESSURE_SENSOR_CALIBRATION_LOW_ADC,
        PRESSURE_SENSOR_CALIBRAION_HIGH_ADC,
        PRESSURE_SENSOR_CALIBRATION_LOW_KPA,
        PRESSURE_SENSOR_CALIBRATION_HIGH_KPA
    );
}

void CoolerSystem::_pollCoolantLevel()
{
    coolantLevel = analogRead(coolantLevelPin) > 800;
}

void CoolerSystem::_pollFlowRate()
{
    static uint lastPulse = 0;
    static uint pulseInterval = 0;
    static ABounce flowRateInput = ABounce(flowRatePin, 3, 1200);
    if (flowRateInput.update() && flowRateInput.risingEdge()) {
        pulseInterval = micros() - lastPulse;
        lastPulse += pulseInterval;
        flowRate = MLPM_MAGIC_NUMBER / pulseInterval;
        digitalWrite(flowPulsePin, !digitalRead(flowPulsePin));
    }
    // if we haven't seen a pulse in a long time
    // reset flow rate to 0
    if (micros() - lastPulse > 500000) {
        flowRate = 0;
        LOG_DEBUG("Flow sensor timeout");
    }
}

void CoolerSystem::_pollNTCSensors()
{
    chillerInletTemp = inletNTC.temperature();
    chillerOutletTemp = outletNTC.temperature();
}

void CoolerSystem::runChillerPump()
{
    static uint32_t pumpStartTime = 0;
    if (systemStatus < CoolerSystemStatus::PRECHILL) {
        if (digitalRead(chillerPumpPin)) {
            LOG_INFO("Stopping chiller pump");
        }
        digitalWrite(chillerPumpPin, LOW); 
        return;
    }

    if (!digitalRead(chillerPumpPin)) {
        pumpStartTime = millis();
        digitalWrite(chillerPumpPin, HIGH);
        LOG_INFO("Starting chiller pump");
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
    if (chillerOutletTemp <= COMPRESSOR_UNDER_TEMP_CUTOFF && !undertempCutoff) {
        undertempCutoff = true;
        LOG_WARN("Compressor stopped due to under temp cut-off, current temp", chillerOutletTemp);
    }
    if (chillerOutletTemp >= COMPRESSOR_RESUME_PID_CONTROL_TEMP && undertempCutoff) {
        undertempCutoff = false;
        LOG_INFO("Resume compressor automated control");
    }

    uint16_t newCompressorValue = ADC_MAX;
    if (systemStatus < CoolerSystemStatus::PRECHILL || undertempCutoff) {
        digitalWrite(systemEnablePin, LOW);
        newCompressorValue = ADC_MAX;
    } else {
        digitalWrite(systemEnablePin, HIGH);
        newCompressorValue = compressorOutputValue;
        // LOG_DEBUG("Compressor input temp:", compressorInputTemp, "target temp:", compressorTempTarget, "output value", compressorOutputValue);
    }
    compressorValue = newCompressorValue;
    analogWrite(compressorPin, compressorValue);
}

void CoolerSystem::runCoolshirtPump()
{
    if (systemStatus == CoolerSystemStatus::PUMP_LOW) {
        coolshirtPumpValue = 3280;
    } else if (systemStatus == CoolerSystemStatus::PUMP_MEDIUM) {
        coolshirtPumpValue = 5440;
    } else if (systemStatus == CoolerSystemStatus::PUMP_HIGH) {
        coolshirtPumpValue = ADC_MAX;
    } else {
        coolshirtPumpValue = 0;
    }
    analogWrite(coolshirtPumpPin, coolshirtPumpValue);
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
    static Metro pollTimer = Metro(100);
    static Metro displayInfoTimer = Metro(1000);

    _pollFlowRate();
    if (pollTimer.check()) {
        // so as long as we call _pollSwitchPosition before doing any actions
        // we can rely on switch position to accurately give a system state
        _pollSystemStatus();

        _pollSystemPressure();
        check(systemPressure < OVERPRESSURE_THRESHOLD_KPA, SystemFault::SYSTEM_OVER_PRESSURE);

        _pollCoolantLevel();
        check(coolantLevel, SystemFault::LOW_COOLANT);

        _pollNTCSensors();

        // begin actions
        runChillerPump();
        runCompressor();
        runCoolshirtPump();
    }
    compressorPID.Compute();

    if (displayInfoTimer.check()) {
        bool systemEnabled = digitalRead(systemEnablePin);
        bool chillerEnabled = digitalRead(chillerPumpPin);
        LOG_INFO("----------------- Cooler Statistics -------------------");
        LOG_INFO("Inputs ------------------------------------------------");
        LOG_INFO("  Inlet Temperature:             ", chillerInletTemp, "C");
        LOG_INFO("  Outlet Temperature:            ", chillerOutletTemp, "C");
        LOG_INFO("  System Pressure:               ", systemPressure, "kPa");
        LOG_INFO("  Flow Rate:                     ", flowRate, "mL/min");
        LOG_INFO("  Coolant Level:                 ", coolantLevel ? "OK" : "LOW");
        LOG_INFO("  Switch Position:               ", CoolerSystemStatusToString(_getSwitchPosition()), " (", analogRead(switchPin), ")");
        LOG_INFO("Outputs -----------------------------------------------");
        LOG_INFO("  System Enabled:                ", (systemEnabled) ? "ON" : "OFF");
        LOG_INFO("  Chiller Pump:                  ", (chillerEnabled) ? "ON" : "OFF");
        LOG_INFO("  Compressor PWM:                ", map(compressorValue, 0, ADC_MAX, 100, 0), "%");
        LOG_INFO("  Coolshirt PWM:                 ", map(coolshirtPumpValue, 0, ADC_MAX, 0, 100), "%");
        LOG_INFO("  Undertemp Cutoff:              ", undertempCutoff ? "CUTOFF" : "OK");
        LOG_INFO("Faults ------------------------------------------------");
        LOG_INFO("  LOW_COOLANT (", (uint8_t)SystemFault::LOW_COOLANT, "):             ", (_systemFault & (byte)SystemFault::LOW_COOLANT) ? "ALARM" : "OK");
        LOG_INFO("  FLOW_RATE_LOW (", (uint8_t)SystemFault::FLOW_RATE_LOW, "):           ", (_systemFault & (byte)SystemFault::FLOW_RATE_LOW) ? "ALARM" : "OK");
        LOG_INFO("  SYSTEM_OVER_PRESSURE (", (uint8_t)SystemFault::SYSTEM_OVER_PRESSURE, "):    ", (_systemFault & (byte)SystemFault::SYSTEM_OVER_PRESSURE) ? "ALARM" : "OK");
        LOG_INFO("  SYSTEM_STARTUP (", (uint8_t)SystemFault::SYSTEM_STARTUP, "):        ", (_systemFault & (byte)SystemFault::SYSTEM_STARTUP) ? "ALARM" : "OK");
        LOG_INFO("-------------------------------------------------------");
    }
};

byte CoolerSystem::systemFault() {
    return _systemFault;
}

void CoolerSystem::getCANMessage(CAN_message_t &msg)
{
    msg.id = CANID_COOLER_SYSTEM;
    msg.len = 8;
    msg.flags = {};

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
    //      |   [2,1,0]: switch position
    // 7    | system faults (seen not necessarily current state)

    msg.buf[0] = uint8_t((chillerInletTemp + 15) / 0.25);
    msg.buf[1] = uint8_t((chillerOutletTemp + 15) / 0.25);
    msg.buf[2] = (uint8_t)(flowRate / 10);
    msg.buf[3] = systemPressure / 4;
    msg.buf[4] = (uint8_t)map(compressorValue, 0, ADC_MAX, 0, 254);
    msg.buf[5] = (uint8_t)map(coolshirtPumpValue, 0, ADC_MAX, 0, 254);
    msg.buf[6] = 0;
    if (digitalRead(systemEnablePin)) msg.buf[6] |= 0x80;
    if (digitalRead(chillerPumpPin)) msg.buf[6] |= 0x40;
    if (_systemFault) msg.buf[6] |= 0x20;
    if (coolantLevel) msg.buf[6] |= 0x10;
    if (undertempCutoff) msg.buf[6] |= 0x08;
    msg.buf[6] |= (0x07 & (uint8_t)systemStatus);
    msg.buf[7] = (uint8_t)_systemFault;
}
