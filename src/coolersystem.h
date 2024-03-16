#pragma once

#include "Arduino.h"

#include <DebugLog.h>
#include <SPI.h>
#include <Bounce.h>
#include <PID_v1.h>
#include <Metro.h>
#include <FlexCAN.h>

#include "constants.h"
#include "pwm.h"
#include "ntc.h"
#include "calibratedadc.h"
#include "switchadc.h"
#include "voltagemonitor.h"
#include "flowsensor.h"
#include "compressorfault.h"
#include "ntclogger.h"
#include "looptimer.h"
#include "stringformat.h"
#include "timer.h"

#define OVERPRESSURE_THRESHOLD_KPA 250 // operating pressure ~170 - 200kPa
#define PRESSURE_SENSOR_CALIBRATION_LOW_ADC 5674 // 0.5V = 0psig = 101kPa
#define PRESSURE_SENSOR_CALIBRATION_HIGH_ADC 51063  // 4.5V = 100psig = 791kPa
#define PRESSURE_SENSOR_CALIBRATION_LOW_KPA 101
#define PRESSURE_SENSOR_CALIBRATION_HIGH_KPA 791

#define CURRENT_SENSOR_CALIBRATION_LOW_ADC 5660 // 0.5V = 0A
#define CURRENT_SENSOR_CALIBRATION_HIGH_ADC 50942  // 4.5V = 50A
#define CURRENT_SENSOR_CALIBRATION_LOW_AMPS 0
#define CURRENT_SENSOR_CALIBRATION_HIGH_AMPS 50

#define FLOW_SENSOR_PULSES_PER_SECOND 7.5
#define FLOW_RATE_MIN_THRESHOLD 1000 // mL/min
#define FLOW_RATE_STARTUP_TIME 5000 // ms allowed until the flow rate must be above threshold
#define FLOW_RATE_MISSING_PULSE_TIME 1000 // ms allowed since the last pulse seen
#define SPECIFIC_HEAT 4033 // J/kgK of chiller fluid (90% water / 10% IPA @ 3C)

#define COMPRESSOR_UNDER_TEMP_CUTOFF_HIGH 3.0
#define COMPRESSOR_RESTART_TEMP_HIGH 6.0
#define COMPRESSOR_UNDER_TEMP_CUTOFF_MED 7.0
#define COMPRESSOR_RESTART_TEMP_MED 10.0
#define COMPRESSOR_UNDER_TEMP_CUTOFF_LOW 11.0
#define COMPRESSOR_RESTART_TEMP_LOW 14.0
#define EVAPORATOR_OUTLET_PANIC_TEMPERATURE 0.0

// valid range of compressor speed output is 4.5V = 50%, 9V = 100%
#define COMPRESSOR_MIN_SPEED_RATIO 0.5
#define COMPRESSOR_MAX_SPEED_RATIO 1.0
#define COMPRESSOR_DEFAULT_SPEED 0.75
#define COMPRESSOR_SPEED_RATIO_TO_ANALOG (9 / (3.3 * 3.717) * ADC_MAX)
#define COMPRESSOR_MIN_COOLDOWN_MS 60000
#define PID_KP 0.5
#define PID_KI 0.2
#define PID_KD 0

// time to acquire data and stabilize before doing anything
#define STARTUP_STABILIZATION_SAMPLES 100

#define NTC_DEBUG 0
#define FLOW_DEBUG 1
#define UPDATE_STATE_TIMER_MS 100
#define DISPLAY_INFO_MS 2000

enum class CoolerSystemStatus {
    STARTUP         = -2,
    REQUIRES_RESET  = -1,
    RESET           = 0,
    PRECHILL        = 1,
    PUMP_LOW        = 2,
    PUMP_MEDIUM     = 3,
    PUMP_HIGH       = 4
};

constexpr const char* CoolerSystemStatusToString(CoolerSystemStatus css)
{
    switch (css)
    {
        case CoolerSystemStatus::STARTUP: return "STARTUP";
        case CoolerSystemStatus::REQUIRES_RESET: return "REQUIRES_RESET";
        case CoolerSystemStatus::RESET: return "RESET";
        case CoolerSystemStatus::PRECHILL: return "PRECHILL";
        case CoolerSystemStatus::PUMP_LOW: return "PUMP_LOW";
        case CoolerSystemStatus::PUMP_MEDIUM: return "PUMP_MEDIUM";
        case CoolerSystemStatus::PUMP_HIGH: return "PUMP_HIGH";
        default: return "UNKNOWN";
    }
}

constexpr const char* CoolerSwitchPositionToString(CoolerSwitchPosition csp)
{
    switch (csp)
    {
        case CoolerSwitchPosition::RESET: return "RESET";
        case CoolerSwitchPosition::PRECHILL: return "PRECHILL";
        case CoolerSwitchPosition::PUMP_LOW: return "PUMP_LOW";
        case CoolerSwitchPosition::PUMP_MEDIUM: return "PUMP_MEDIUM";
        case CoolerSwitchPosition::PUMP_HIGH: return "PUMP_HIGH";
        default: return "UNKNOWN";
    }
}

enum class SystemFault {
    SYSTEM_OK  = 0,
    LOW_COOLANT = 1,
    FLOW_RATE_LOW = 2,
    SYSTEM_OVER_PRESSURE = 4,
    SYSTEM_UNDERVOLT = 8,
    SYSTEM_OVERVOLT = 16,
    COMPRESSOR_FAULT = 32,
};

constexpr const char* SystemFaultToString(SystemFault sf)
{
    switch (sf)
    {
        case SystemFault::SYSTEM_OK: return "SYSTEM_OK";
        case SystemFault::LOW_COOLANT: return "LOW_COOLANT";
        case SystemFault::FLOW_RATE_LOW: return "FLOW_RATE_LOW";
        case SystemFault::SYSTEM_OVER_PRESSURE: return "SYSTEM_OVER_PRESSURE";
        case SystemFault::SYSTEM_UNDERVOLT: return "SYSTEM_UNDERVOLT";
        case SystemFault::SYSTEM_OVERVOLT: return "SYSTEM_OVERVOLT";
        case SystemFault::COMPRESSOR_FAULT: return "COMPRESSOR_FAULT";
        default: return "UNKNOWN";
    }
}

struct CoolerSystemData {
    byte fault;
    bool coolantLevel;
    uint16_t systemPressure;
    float compressorCurrent;
    float evaporatorInletTemp;
    float evaporatorOutletTemp;
    float condenserInletTemp;
    float condenserOutletTemp;
    float ambientTemp;
    uint16_t flowRate;
    float compressorSpeed;
};

class CoolerSystem {
private:
    // inputs
    SwitchADC switchADC;
    CalibratedADC pressureSensor;
    CalibratedADC currentSensor;
    CompressorFault compressorFault;
    uint8_t coolantLevelPin;
    uint8_t compressorSpeedPin;
    FlowSensor flowSensor;
    PrecisionNTC evaporatorInletNTC, evaporatorInletA10, evaporatorOutletNTC;
    NTC condenserInletNTC, condenserOutletNTC;
    NTC ambientNTC;

    // outputs
    PWMOutput coolshirtPWM, chillerPumpPWM, systemEnableOutput;

    // internal state
    Bounce coolantLevelBounce;
    VoltageMonitor voltageMonitor;
    byte _systemFault { (byte)SystemFault::SYSTEM_OK };
    CoolerSystemStatus systemStatus { CoolerSystemStatus::STARTUP };
    CoolerSwitchPosition switchPosition { CoolerSwitchPosition::UNKNOWN };

    LoopTimer loopTimer;
    uint32_t sampleCounter { 0 };
    MetroTimer msTick = { MetroTimer(1) };
    MetroTimer updateStateTimer = { MetroTimer(UPDATE_STATE_TIMER_MS) };
    MetroTimer displayInfoTimer = { MetroTimer(DISPLAY_INFO_MS) };
    uint32_t pumpStartTime = { 0 };
    NTCLogger ntcLogger;

    // sensor values
    CompressorFaultCode compressorFaultCode { CompressorFaultCode::OK };
    uint16_t flowRate { 0 };
    uint8_t lastLoggedFlowPulse { 0 };
    bool coolantLevel { false };
    uint16_t systemPressure { 0 };
    float compressorCurrent { 0 };
    float evaporatorOutletTemp { -100.0 };
    float condenserInletTemp { -100.0 };
    float condenserOutletTemp { -100.0 };
    float ambientTemp { -100.0 };
    float coolingPower { 0 };
    uint32_t compressorShutoffTime { 0 };

    // PID control inputs/outputs
    double evaporatorInletTemp { -100.0 };
    double compressorSpeed { 0 };
    double compressorTempTarget { 5 };
    bool undertempCutoff { false };
    PID compressorPID { PID(
        &evaporatorInletTemp, &compressorSpeed, &compressorTempTarget,
        PID_KP, PID_KI, PID_KD, P_ON_M, REVERSE
    )};

    // pollers/updaters
    void pollCoolantLevel();

    // executors
    void runChillerPump();
    void runCompressor();
    void shutdownCompressor();
    void startupCompressor();
    void runCoolshirtPump();
    void check(bool assertionResult, SystemFault fault);

    void acquireSamples();
    void updateState();
    void updateOutputs();
    void displayInfo();

public:
    CoolerSystem(
        uint8_t _switchPin,
        uint8_t _switchADCNum,
        uint8_t _coolantLevelPin,
        uint8_t _flowRatePin,
        uint8_t _pressureSensorPin,
        uint8_t _pressureSensorADCNum,
        uint8_t _currentSensorPin,
        uint8_t _currentSensorADCNum,
        uint8_t _compressorLDRPin,
        uint8_t _evaporatorInletNtcPin,
        uint8_t _evaporatorOutletNtcPin,
        uint8_t _condenserInletNtcPin,
        uint8_t _condenserOutletNtcPin,
        uint8_t _ambientNtcPin,
        uint8_t _ntcADCNum,
        uint8_t _compressorSpeedPin,
        uint8_t _chillerPumpPin,
        uint8_t _coolshirtPumpPin,
        uint8_t _systemEnablePin,
        uint8_t sys12vPin,
        uint8_t sys12vADCNum,
        uint8_t sys5vPin,
        uint8_t sys5vADCNum,
        uint8_t sys3v3Pin,
        uint8_t sys3v3ADCNum,
        uint8_t sysp3v3Pin,
        uint8_t sysp3v3ADCNum
    )
        : switchADC { SwitchADC(_switchPin, _switchADCNum) }
        , pressureSensor { CalibratedADC(_pressureSensorPin, _pressureSensorADCNum) }
        , currentSensor { CalibratedADC(_currentSensorPin, _currentSensorADCNum) }
        , compressorFault { CompressorFault(_compressorLDRPin) }
        , coolantLevelPin { _coolantLevelPin }
        , compressorSpeedPin { _compressorSpeedPin }
        , flowSensor { FlowSensor(_flowRatePin, FLOW_SENSOR_PULSES_PER_SECOND, FLOW_RATE_MISSING_PULSE_TIME) }
        , evaporatorInletNTC { PrecisionNTC(_evaporatorInletNtcPin, _ntcADCNum, 15000) }
        , evaporatorInletA10 { PrecisionNTC(NTC_EVAPORATOR_DIFF_1, _ntcADCNum, 15000) }
        , evaporatorOutletNTC { PrecisionNTC(_evaporatorOutletNtcPin, _ntcADCNum, 15000) }
        , condenserInletNTC { NTC(_condenserInletNtcPin, _ntcADCNum, 6800) }
        , condenserOutletNTC { NTC(_condenserOutletNtcPin, _ntcADCNum, 6800) }
        , ambientNTC { PrecisionNTC(_ambientNtcPin, _ntcADCNum, 6800) }
        , coolshirtPWM { PWMOutput(_coolshirtPumpPin) }
        , chillerPumpPWM { PWMOutput(_chillerPumpPin) }
        , systemEnableOutput { PWMOutput(_systemEnablePin) }
        , coolantLevelBounce { Bounce(coolantLevelPin, 10) }
        , voltageMonitor { VoltageMonitor(sys12vPin, sys12vADCNum, sys5vPin, sys5vADCNum, sys3v3Pin, sys3v3ADCNum, sysp3v3Pin, sysp3v3ADCNum) }
    {
    };
    void setup();
    void loop();
    void getCANMessage(CAN_message_t &msg);
    void getLogMessage(StringFormatCSV& format);
    byte systemFault();
    void getSystemData(CoolerSystemData &data);
    unsigned long lastFlowPulseMicros();
    bool hasStarted();
};
