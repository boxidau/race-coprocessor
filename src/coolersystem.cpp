#include "coolersystem.h"

void CoolerSystem::setup()
{
    // setup pin modes and initial states
    pinMode(switchPin, INPUT);
    pinMode(coolantLevelPin, INPUT);
    pinMode(flowRatePin, INPUT);
    pinMode(pressureSensorPin, INPUT);
    pinMode(compressorPin, OUTPUT);
    pinMode(chillerPumpPin, OUTPUT);
    pinMode(coolshirtPumpPin, OUTPUT);
    pinMode(systemEnablePin, OUTPUT);
    digitalWriteFast(compressorPin, LOW);
    digitalWriteFast(chillerPumpPin, LOW);
    digitalWriteFast(coolshirtPumpPin, LOW);
    digitalWriteFast(systemEnablePin, LOW);
    Thermocouple::setup();
    compressorPID.SetOutputLimits(0, 1023);
    compressorPID.SetMode(AUTOMATIC);
};

CoolerSwitchPosition CoolerSystem::_getSwitchPosition()
{
    const uint16_t switchADCValue = analogRead(switchPin);
    // LOG_DEBUG("Position switch ADC value", switchADCValue);
    if (switchADCValue < 100) {
        systemFault = SystemFault::SYSTEM_OK;
        return CoolerSwitchPosition::RESET;
    }
    if (systemFault > SystemFault::SYSTEM_OK) return CoolerSwitchPosition::REQUIRES_RESET;
    if (switchADCValue < 300) return CoolerSwitchPosition::PRECHILL;
    if (switchADCValue < 500) return CoolerSwitchPosition::PUMP_LOW;
    if (switchADCValue < 700) return CoolerSwitchPosition::PUMP_MEDIUM;
    if (switchADCValue >= 700) return CoolerSwitchPosition::PUMP_HIGH;

    LOG_ERROR("Unknown ADC value", switchADCValue, "for switch position");
    return CoolerSwitchPosition::REQUIRES_RESET;
}

void CoolerSystem::_pollSwitchPosition()
{
    const CoolerSwitchPosition _newSwitchPosition = _getSwitchPosition();
    if (_newSwitchPosition != switchPosition) {
        LOG_INFO("Switch position changed", CoolerSwitchPositionToString(switchPosition), "to", CoolerSwitchPositionToString(_newSwitchPosition));
        switchPosition = _newSwitchPosition;
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
    static Bounce coolantLevelInput = Bounce(coolantLevelPin, 100);
    if (coolantLevelInput.update()) {
        coolantLevel = coolantLevelInput.read();
    }
    if (MOCK_DATA) coolantLevel = true;
}

void CoolerSystem::_pollFlowRate()
{
    static uint lastPulse = 0;
    static uint pulseInterval = 0;
    static Bounce flowRateInput = Bounce(flowRatePin, 3);
    if (flowRateInput.update() && flowRateInput.risingEdge()) {
        pulseInterval = micros() - lastPulse;
        lastPulse += pulseInterval;
        flowRate = MLPM_MAGIC_NUMBER / pulseInterval;
    }
    if (MOCK_DATA) flowRate = 2000;
}

void CoolerSystem::runChillerPump()
{
    static uint32_t pumpStartTime = 0;
    if (switchPosition < CoolerSwitchPosition::PRECHILL) {
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
    if (flowRate <= FLOW_RATE_MIN_THRESHOLD && pumpRunTime >= FLOW_RATE_MIN_TIME_MS_THRESHOLD) {
        // we haven't seen enough flow after some amount of time
        // maybe the pump is broken
        LOG_ERROR("Pump started", pumpRunTime, "ms ago, inadequate pump flow, panic!");
        panic(SystemFault::FLOW_RATE_LOW);
    }
};

void CoolerSystem::runCoolshirtPump()
{
    uint16_t newCoolshirtPumpValue = 0;
    if (switchPosition == CoolerSwitchPosition::PUMP_LOW) newCoolshirtPumpValue = 410;
    if (switchPosition == CoolerSwitchPosition::PUMP_MEDIUM) newCoolshirtPumpValue = 680;
    if (switchPosition == CoolerSwitchPosition::PUMP_HIGH) newCoolshirtPumpValue = 1023;
    if (newCoolshirtPumpValue != coolshirtPumpValue) {
        LOG_INFO("Setting coolshirt pump PWM to", map(newCoolshirtPumpValue, 0, 1023, 0, 100), "%");
    }
    coolshirtPumpValue = newCoolshirtPumpValue;
    analogWrite(coolshirtPumpPin, coolshirtPumpValue);
}

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

    uint16_t newCompressorValue = 0;
    if (switchPosition < CoolerSwitchPosition::PRECHILL || undertempCutoff) {
        newCompressorValue = 0;
    } else {
        newCompressorValue = compressorOutputValue;
        LOG_DEBUG("Compressor input temp:", compressorInputTemp, "target temp:", compressorTempTarget, "output value", compressorOutputValue);
    }
    if (compressorValue != newCompressorValue) {
        LOG_INFO("Updating compressor PWM to", map(newCompressorValue, 0, 1023, 0, 100));
    }
    compressorValue = newCompressorValue;
    analogWrite(compressorPin, compressorValue);
}

void CoolerSystem::panic(SystemFault fault)
{
    if (systemFault != fault) {
        LOG_ERROR("Cooler system panic with fault code:", SystemFaultToString(fault));
    }
    systemFault = fault;
    _pollSwitchPosition();
}

void CoolerSystem::loop()
{
    static Metro pollTimer = Metro(10);

    if (pollTimer.check()) {
        // so as long as we call _pollSwitchPosition before doing any actions
        // we can rely on switch position to accurately give a system state
        _pollSwitchPosition();
        _pollSystemPressure();
        _pollFlowRate();
        _pollCoolantLevel();
        if (!coolantLevel) {
            panic(SystemFault::LOW_COOLANT);
        }
        Thermocouple::readData(thermocoupleCSPin, evaporatorTemp);
        if (evaporatorTemp.error) {
            panic(SystemFault::THERMOCOUPLE_ERROR);
        }

        // begin actions
        digitalWrite(systemEnablePin, switchPosition > CoolerSwitchPosition::RESET);
        runChillerPump();
        runCompressor();
        runCoolshirtPump();
    }
    compressorPID.Compute();
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
    msg.buf[4] = (uint8_t)map(compressorValue, 0, 1023, 0, 254);
    msg.buf[5] = (uint8_t)map(coolshirtPumpValue, 0, 1023, 0, 254);
    msg.buf[6] = 0;
    if (digitalRead(systemEnablePin)) msg.buf[6] |= 0x80;
    if (digitalRead(chillerPumpPin)) msg.buf[6] |= 0x40;
    if (systemFault > SystemFault::SYSTEM_OK) msg.buf[6] |= 0x20;
    if (coolantLevel) msg.buf[6] |= 0x10;
    if (undertempCutoff) msg.buf[6] |= 0x08;
    msg.buf[6] |= (0x07 & (uint8_t)switchPosition);
    msg.buf[7] = (uint8_t)systemFault;
}
