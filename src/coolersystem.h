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

#define OVERPRESSURE_THRESHOLD_KPA 230
#define PRESSURE_SENSOR_CALIBRATION_LOW_ADC 5674 // 0.5V = 0psig = 101kPa
#define PRESSURE_SENSOR_CALIBRAION_HIGH_ADC 51063  // 4.5V = 100psig = 791kPa
#define PRESSURE_SENSOR_CALIBRATION_LOW_KPA 101
#define PRESSURE_SENSOR_CALIBRATION_HIGH_KPA 791

#define FLOW_SENSOR_PULSES_PER_SECOND 7.5
#define FLOW_RATE_MIN_THRESHOLD 1000
#define FLOW_RATE_MIN_TIME_MS_THRESHOLD 5000

#define DESIRED_TEMP 5.0
#define COMPRESSOR_UNDER_TEMP_CUTOFF 3.0
#define COMPRESSOR_RESUME_PID_CONTROL_TEMP 7.0
#define COMPRESSOR_MIN_SPEED_RATIO 0.5
#define COMPRESSOR_MAX_SPEED_RATIO 1.0
#define COMPRESSOR_SPEED_RATIO_TO_ANALOG (9 / (3.3 * 3.717) * 4095)

#define NTC_DEBUG 0

enum class CoolerSystemStatus {
    REQUIRES_RESET = 0,
    RESET          = 1,
    PRECHILL       = 2,
    PUMP_LOW       = 3,
    PUMP_MEDIUM    = 4,
    PUMP_HIGH      = 5
};

constexpr CoolerSystemStatus CoolerSwitchPositionToStatus(CoolerSwitchPosition csp) {
    switch (csp)
    {
        case CoolerSwitchPosition::RESET: return CoolerSystemStatus::RESET;
        case CoolerSwitchPosition::PRECHILL: return CoolerSystemStatus::PRECHILL;
        case CoolerSwitchPosition::PUMP_LOW: return CoolerSystemStatus::PUMP_LOW;
        case CoolerSwitchPosition::PUMP_MEDIUM: return CoolerSystemStatus::PUMP_MEDIUM;
        case CoolerSwitchPosition::PUMP_HIGH: return CoolerSystemStatus::PUMP_HIGH;
        default:
            LOG_ERROR("Invalid switch position", int(csp));
            return CoolerSystemStatus::RESET;
    }
}

constexpr const char* CoolerSystemStatusToString(CoolerSystemStatus css)
{
    switch (css)
    {
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
    SYSTEM_STARTUP = 32,
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
        case SystemFault::SYSTEM_STARTUP: return "SYSTEM_STATUP";
        default: return "UNKNOWN";
    }
}

struct CoolerSystemData {
    byte fault;
    bool coolantLevel;
    uint16_t systemPressure;
    double evaporatorInletTemp;
    double evaporatorOutletTemp;
    double condenserInletTemp;
    double condenserOutletTemp;
    double ambientTemp;
    uint16_t flowRate;
    double compressorSpeed;
};

class CoolerSystem {
private:
    // inputs
    SwitchADC switchADC;
    CalibratedADC pressureSensor;
    uint8_t coolantLevelPin;
    uint8_t compressorSpeedPin;
    FlowSensor flowSensor;
    NTC evaporatorInletNTC, evaporatorOutletNTC, condenserInletNTC, condenserOutletNTC, ambientNTC;

    // outputs
    PWMOutput coolshirtPWM, chillerPumpPWM, systemEnableOutput;

    // internal state
    Bounce coolantLevelBounce;
    VoltageMonitor& voltageMonitor;
    byte _systemFault { (byte)SystemFault::SYSTEM_STARTUP };
    CoolerSystemStatus systemStatus { CoolerSystemStatus::REQUIRES_RESET };
    CoolerSwitchPosition switchPosition { CoolerSwitchPosition::UNKNOWN };

    Metro pollTimer { Metro(100) };
    Metro displayInfoTimer { Metro(NTC_DEBUG ? 500 : 2000) };
    Metro msTick = { Metro(1) };
    uint32_t pumpStartTime = { 0 };

    // sensor values
    uint16_t flowRate { 0 };
    bool coolantLevel { false };
    uint16_t systemPressure { 0 };
    double evaporatorInletTemp { -100.0 };
    double evaporatorOutletTemp { -100.0 };
    double condenserInletTemp { -100.0 };
    double condenserOutletTemp { -100.0 };
    double ambientTemp { -100.0 };
    bool chillerPumpRunning { false };

    // output states
    // for logging/canbus output
    // systemEnable and chillerPump can just digitalRead their own state
    uint16_t coolshirtPumpValue { 0 };
    uint16_t compressorValue { 0 };

    double compressorSpeed { 0 };
    double compressorTempTarget { DESIRED_TEMP };
    bool undertempCutoff { false };
    const double Kp=1.5e-1, Ki=1e-3, Kd=0;
    PID compressorPID { PID(
        &evaporatorOutletTemp, &compressorSpeed, &compressorTempTarget,
        Kp, Ki, Kd, P_ON_M, REVERSE
    )};

    // getters
    // will return REQUIRES_RESET until the switch has visited the RESET position at least once
    // in a panic condition the need to re-visit the reset is enabled
    CoolerSystemStatus _getSwitchPosition();

    // pollers/updaters
    void _pollSystemStatus();
    void _pollCoolantLevel();

    // executors
    void runChillerPump();
    void runCompressor();
    void runCoolshirtPump();
    void check(bool assertionResult, SystemFault fault);

public:
    CoolerSystem(
        uint8_t _switchPin,
        uint8_t _switchADCNum,
        uint8_t _coolantLevelPin,
        uint8_t _flowRatePin,
        uint8_t _pressureSensorPin,
        uint8_t _pressureSensorADCNum,
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
        VoltageMonitor &_voltageMonitor
    )
        : switchADC { SwitchADC(_switchPin, _switchADCNum) }
        , pressureSensor { CalibratedADC(_pressureSensorPin, _pressureSensorADCNum) }
        , coolantLevelPin { _coolantLevelPin }
        , compressorSpeedPin { _compressorSpeedPin }
        , flowSensor { FlowSensor(_flowRatePin, FLOW_SENSOR_PULSES_PER_SECOND) }
        , evaporatorInletNTC { NTC(_evaporatorInletNtcPin, _ntcADCNum, 6800) }
        , evaporatorOutletNTC { NTC(_evaporatorOutletNtcPin, _ntcADCNum, 6800) }
        , condenserInletNTC { NTC(_condenserInletNtcPin, _ntcADCNum, 15000) }
        , condenserOutletNTC { NTC(_condenserOutletNtcPin, _ntcADCNum, 15000) }
        , ambientNTC { NTC(_ambientNtcPin, _ntcADCNum, 6800) }
        , coolshirtPWM { PWMOutput(_coolshirtPumpPin) }
        , chillerPumpPWM { PWMOutput(_chillerPumpPin) }
        , systemEnableOutput { PWMOutput(_systemEnablePin) }
        , coolantLevelBounce { Bounce(coolantLevelPin, 40) }
        , voltageMonitor { _voltageMonitor }
    {
    };
    void setup();
    void loop();
    void getCANMessage(CAN_message_t &msg);
    byte systemFault();
    void getSystemData(CoolerSystemData &data);
};
