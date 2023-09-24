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

    Thermocouple::setup();
    compressorPID.SetOutputLimits(0, 8191);
    compressorPID.SetMode(AUTOMATIC);
};

CoolerSystemStatus CoolerSystem::_getSwitchPosition()
{
    const uint16_t switchADCValue = analogRead(switchPin);
    // LOG_INFO("Position switch ADC value", switchADCValue);
    if (switchADCValue < 760) return CoolerSystemStatus::RESET;
    if (switchADCValue < 1136) return CoolerSystemStatus::PRECHILL;
    if (switchADCValue < 1360) return CoolerSystemStatus::PUMP_LOW;
    if (switchADCValue < 1520) return CoolerSystemStatus::PUMP_MEDIUM;
    if (switchADCValue < 1640) return CoolerSystemStatus::PUMP_HIGH;
    // out of range?
    return systemStatus;
}

void CoolerSystem::_pollSystemStatus()
{
    CoolerSystemStatus _newSwitchPosition = _getSwitchPosition();

    if (systemFault > 0 && _newSwitchPosition != CoolerSystemStatus::RESET) {
        _newSwitchPosition = CoolerSystemStatus::REQUIRES_RESET;
    }
    if (_newSwitchPosition == CoolerSystemStatus::RESET) {
        systemFault = 0;
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
    // PID controller for compressor    
    compressorInputTemp = evaporatorTemp.temperature;

    // undertemp check
    // this isn't a panic condition
    // but we are going to shut down the compressor until temp
    // goes back above a safe value
    if (evaporatorTemp.temperature <= COMPRESSOR_UNDER_TEMP_CUTOFF && !undertempCutoff) {
        undertempCutoff = true;
        LOG_WARN("Compressor stopped due to under temp cut-off, current temp", evaporatorTemp.temperature);
    }
    if (evaporatorTemp.temperature >= COMPRESSOR_RESUME_PID_CONTROL_TEMP && undertempCutoff) {
        undertempCutoff = false;
        LOG_INFO("Resume compressor automated control");
    }

    uint16_t newCompressorValue = 8191;
    if (systemStatus < CoolerSystemStatus::PRECHILL || undertempCutoff) {
        newCompressorValue = 8191;
    } else {
        newCompressorValue = compressorOutputValue;
        // LOG_DEBUG("Compressor input temp:", compressorInputTemp, "target temp:", compressorTempTarget, "output value", compressorOutputValue);
    }
    compressorValue = newCompressorValue;
    analogWrite(compressorPin, compressorValue);
}

void CoolerSystem::runCoolshirtPump()
{
    if (systemStatus == CoolerSystemStatus::PUMP_LOW) coolshirtPumpValue = 3280;
    if (systemStatus == CoolerSystemStatus::PUMP_MEDIUM) coolshirtPumpValue = 5440;
    if (systemStatus == CoolerSystemStatus::PUMP_HIGH) coolshirtPumpValue = 8191;
    analogWrite(coolshirtPumpPin, coolshirtPumpValue);
}

void CoolerSystem::check(bool assertionResult, SystemFault fault)
{
    if (!assertionResult) {
        // fault code raised, assertion is false
        if (!(systemFault & (byte)fault)) {
            LOG_ERROR("Fault code raised", SystemFaultToString(fault));
        }
        systemFault |= (byte)fault;
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

        Thermocouple::readData(thermocoupleCSPin, evaporatorTemp);
        // assertPanic(!evaporatorTemp.error, SystemFault::THERMOCOUPLE_ERROR);

        // begin actions
        digitalWrite(systemEnablePin, systemStatus > CoolerSystemStatus::RESET);
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
        LOG_INFO("  Thermocouple:                  ", evaporatorTemp.temperature, "C");
        LOG_INFO("  Ambient Temp:                  ", evaporatorTemp.internalTemperature, "C");
        LOG_INFO("  System Pressure:               ", systemPressure, "kPa");
        LOG_INFO("  Flow Rate:                     ", flowRate, "mL/min");
        LOG_INFO("  Coolant Level:                 ", coolantLevel);
        LOG_INFO("  Switch ADC:                    ", analogRead(switchPin));
        LOG_INFO("Outputs -----------------------------------------------");
        LOG_INFO("  System Enabled:                ", (systemEnabled) ? "ON" : "OFF");
        LOG_INFO("  Chiller Pump:                  ", (chillerEnabled) ? "ON" : "OFF");
        LOG_INFO("  Compressor PWM:                ", map(compressorValue, 0, 8191, 100, 0));
        LOG_INFO("  Coolshirt PWM:                 ", coolshirtPumpValue);
        LOG_INFO("Faults ------------------------------------------------");
        LOG_INFO("  LOW_COOLANT (", (uint8_t)SystemFault::LOW_COOLANT, "):             ", (systemFault & (byte)SystemFault::LOW_COOLANT) ? "ALARM" : "OK");
        LOG_INFO("  FLOW_RATE_LOW (", (uint8_t)SystemFault::FLOW_RATE_LOW, "):           ", (systemFault & (byte)SystemFault::FLOW_RATE_LOW) ? "ALARM" : "OK");
        LOG_INFO("  THERMOCOUPLE_ERROR (", (uint8_t)SystemFault::THERMOCOUPLE_ERROR, "):      ", (systemFault & (byte)SystemFault::THERMOCOUPLE_ERROR) ? "ALARM" : "OK");
        LOG_INFO("  SWITCH_ADC_OUT_OF_BOUNDS (", (uint8_t)SystemFault::SWITCH_ADC_OUT_OF_BOUNDS, "):", (systemFault & (byte)SystemFault::SWITCH_ADC_OUT_OF_BOUNDS) ? "ALARM" : "OK");
        LOG_INFO("  SYSTEM_OVER_PRESSURE (", (uint8_t)SystemFault::SYSTEM_OVER_PRESSURE, "):   ", (systemFault & (byte)SystemFault::SYSTEM_OVER_PRESSURE) ? "ALARM" : "OK");
        LOG_INFO("  SYSTEM_STARTUP (", (uint8_t)SystemFault::SYSTEM_STARTUP, "):         ", (systemFault & (byte)SystemFault::SYSTEM_STARTUP) ? "ALARM" : "OK");
        LOG_INFO("-------------------------------------------------------");
    }
};

void CoolerSystem::getCANMessage(CAN_message_t &msg)
{
    msg.id = CANID_COOLER_SYSTEM;
    msg.len = 8;
    msg.flags = {};

    // byte | purpose
    // ------------------------------
    // 0    | thermocouple temp MSB (celcius) int16_t
    // 1    | thermocouple temp LSB (celcius) int16_t scaling factor 2^-2
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

    msg.buf[0] = evaporatorTemp.scaledTemperature >> 8;
    msg.buf[1] = (evaporatorTemp.scaledTemperature & 0xFF);
    msg.buf[2] = (uint8_t)(flowRate / 10);
    msg.buf[3] = systemPressure / 4;
    msg.buf[4] = (uint8_t)map(compressorValue, 0, 8191, 0, 254);
    msg.buf[5] = (uint8_t)map(coolshirtPumpValue, 0, 8191, 0, 254);
    msg.buf[6] = 0;
    if (digitalRead(systemEnablePin)) msg.buf[6] |= 0x80;
    if (digitalRead(chillerPumpPin)) msg.buf[6] |= 0x40;
    if (systemFault) msg.buf[6] |= 0x20;
    if (coolantLevel) msg.buf[6] |= 0x10;
    if (undertempCutoff) msg.buf[6] |= 0x08;
    msg.buf[6] |= (0x07 & (uint8_t)systemStatus);
    msg.buf[7] = (uint8_t)systemFault;
}
