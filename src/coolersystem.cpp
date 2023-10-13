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

    analogWriteFrequency(coolshirtPumpPin, 24414);
    analogWriteFrequency(compressorPin, 24414);

    pinMode(flowRatePin, INPUT);
    pinMode(coolantLevelPin, INPUT);
    inletNTC.setup();
    outletNTC.setup();

    compressorPID.SetOutputLimits(0, ADC_MAX);
    compressorPID.SetMode(AUTOMATIC);
};

CoolerSystemStatus CoolerSystem::_getSwitchPosition()
{
    static CoolerSystemStatus switchPosition = CoolerSystemStatus::RESET;
    const uint16_t switchADCValue = switchADC.adc();
    if (switchADCValue < 37000) switchPosition = CoolerSystemStatus::RESET;
    else if (switchADCValue < 51000) switchPosition = CoolerSystemStatus::PRECHILL;
    else if (switchADCValue < 57500) switchPosition = CoolerSystemStatus::PUMP_LOW;
    else if (switchADCValue < 63000) switchPosition = CoolerSystemStatus::PUMP_MEDIUM;
    else if (switchADCValue < 66000) switchPosition = CoolerSystemStatus::PUMP_HIGH;
    return switchPosition;
}

void CoolerSystem::_pollSystemStatus()
{
    CoolerSystemStatus _newSwitchPosition = _getSwitchPosition();
    if (_newSwitchPosition == CoolerSystemStatus::RESET) {
        _systemFault = 0;
    }
    if (_systemFault > 0) {
        _newSwitchPosition = CoolerSystemStatus::REQUIRES_RESET;
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
            pressureSensor.adc(),
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
    if (coolantLevelBounce.update()) {
        coolantLevel = coolantLevelBounce.read();
    }
}

void CoolerSystem::_pollFlowRate()
{
    static uint32_t lastPulse = 0;
    static uint32_t pulseInterval = 0;
    static uint32_t samples[FLOW_SAMPLES];
    static uint8_t idx = 0;
    static Bounce flowRateInput = Bounce(flowRatePin, 1);
    static bool ledStatus = LOW;
    if (flowRateInput.update() && flowRateInput.fallingEdge()) {
        pulseInterval = micros() - lastPulse;
        samples[idx % FLOW_SAMPLES] = pulseInterval;
        idx++;
        lastPulse += pulseInterval;
        uint sum = 0;
        for (uint8_t i = 0; i < FLOW_SAMPLES; i++) {
            sum += samples[i];
        }
        uint16_t avg = (sum / FLOW_SAMPLES);
        flowRate = MLPM_MAGIC_NUMBER / avg;
        ledStatus = !ledStatus;
        digitalWriteFast(flowPulsePin, ledStatus);
    }

    // if we haven't seen a pulse in a long time
    // reset flow rate to 0
    if (micros() - lastPulse > 3000000) {
        flowRate = 0;
        // LOG_ERROR("Flow sensor timeout");
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
    static uint16_t pumpOutput = 0;
    // doesn't use system status since we want to run the pump in specific fault modes
    if (_getSwitchPosition() < CoolerSystemStatus::PRECHILL) {
        pumpOutput = 0;
        analogWrite(chillerPumpPin, 0);
        chillerPumpRunning = false;
        return;
    }

    if (
        (_systemFault & uint8_t(SystemFault::LOW_COOLANT))
        || (_systemFault & uint8_t(SystemFault::SYSTEM_STARTUP))
    ) {
        pumpOutput = 0;
        analogWrite(chillerPumpPin, 0); 
        chillerPumpRunning = false;
        return;
    }

    if (pumpOutput == 0) {
        pumpStartTime = millis();
        pumpOutput = (ADC_MAX / 3) * 2;
        analogWrite(chillerPumpPin, pumpOutput);
        chillerPumpRunning = true;
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
    if (chillerInletTemp <= COMPRESSOR_UNDER_TEMP_CUTOFF && !undertempCutoff) {
        undertempCutoff = true;
        LOG_WARN("Compressor stopped due to under temp cut-off, current temp", chillerInletTemp);
    }
    if (chillerInletTemp >= COMPRESSOR_RESUME_PID_CONTROL_TEMP && undertempCutoff) {
        undertempCutoff = false;
        LOG_INFO("Resume compressor automated control");
    }

    uint16_t newCompressorValue = ADC_MAX;
    if (systemStatus < CoolerSystemStatus::PRECHILL || undertempCutoff) {
        digitalWriteFast(systemEnablePin, LOW);
        newCompressorValue = ADC_MAX;
    } else {
        digitalWriteFast(systemEnablePin, HIGH);
        newCompressorValue = compressorOutputValue;
        // LOG_DEBUG("Compressor input temp:", compressorInputTemp, "target temp:", compressorTempTarget, "output value", compressorOutputValue);
    }
    compressorValue = newCompressorValue;
    analogWrite(compressorPin, compressorValue);
}

void CoolerSystem::runCoolshirtPump()
{
    if (systemStatus == CoolerSystemStatus::PUMP_LOW) {
        coolshirtPumpValue = ADC_MAX / 3;
    } else if (systemStatus == CoolerSystemStatus::PUMP_MEDIUM) {
        coolshirtPumpValue = ADC_MAX / 3 * 2;
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
    static Metro pollTimer = Metro(113);
    static Metro displayInfoTimer = Metro(1000);
    static Metro msTick = Metro(1);

    if (msTick.check()) {
        _pollFlowRate();
        inletNTC.loop();
        outletNTC.loop();
        pressureSensor.loop();
        _pollCoolantLevel();
        switchADC.loop();
    }

    if (pollTimer.check()) {
        // so as long as we call _pollSwitchPosition before doing any actions
        // we can rely on switch position to accurately give a system state
        _pollSystemStatus();

        _pollSystemPressure();
        check(systemPressure < OVERPRESSURE_THRESHOLD_KPA, SystemFault::SYSTEM_OVER_PRESSURE);

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
        LOG_INFO("----------------- Cooler Statistics -------------------");
        LOG_INFO("Inputs ------------------------------------------------");
        LOG_INFO("  Inlet Temperature:             ", chillerInletTemp, "C (",inletNTC.adc(), ")");
        LOG_INFO("  Outlet Temperature:            ", chillerOutletTemp, "C (",outletNTC.adc(), ")");
        LOG_INFO("  System Pressure:               ", systemPressure, "kPa (", pressureSensor.adc(), ")");
        LOG_INFO("  Flow Rate:                     ", flowRate, "mL/min");
        LOG_INFO("  Coolant Level:                 ", coolantLevel ? "OK" : "LOW");
        LOG_INFO("  Switch Position:               ", CoolerSystemStatusToString(_getSwitchPosition()), " (", switchADC.adc(), ")");
        LOG_INFO("Outputs -----------------------------------------------");
        LOG_INFO("  System Enabled:                ", (systemEnabled) ? "ON" : "OFF");
        LOG_INFO("  Chiller Pump:                  ", (chillerPumpRunning) ? "ON" : "OFF");
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
    //      |   [2,1,0]: system status
    // 7    | system faults

    msg.buf[0] = chillerInletTemp > -15 ? uint8_t((chillerInletTemp + 15) / 0.25) : 0xFF;
    msg.buf[1] = chillerOutletTemp > -15 ? uint8_t((chillerOutletTemp + 15) / 0.25): 0xFF;
    msg.buf[2] = (uint8_t)(flowRate / 15);
    msg.buf[3] = systemPressure / 4;
    msg.buf[4] = (uint8_t)map(compressorValue, 0, ADC_MAX, 0, 255);
    msg.buf[5] = (uint8_t)map(coolshirtPumpValue, 0, ADC_MAX, 0, 255);
    msg.buf[6] = 0;
    if (digitalRead(systemEnablePin)) msg.buf[6] |= 0x80;
    if (chillerPumpRunning) msg.buf[6] |= 0x40;
    if (_systemFault) msg.buf[6] |= 0x20;
    if (coolantLevel) msg.buf[6] |= 0x10;
    if (undertempCutoff) msg.buf[6] |= 0x08;
    msg.buf[6] |= (0x07 & (uint8_t)systemStatus);
    msg.buf[7] = (uint8_t)_systemFault;
}
