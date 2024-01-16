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
#include "voltagemonitor.h"
#include "flowsensor.h"

#define FLOW_SENSOR_PULSES_PER_SECOND 7.5

#define OVERPRESSURE_THRESHOLD_KPA 230
#define PRESSURE_SENSOR_CALIBRATION_LOW_ADC 5674 // 2112 // 0.5V = 0psig = 101kPa
#define PRESSURE_SENSOR_CALIBRAION_HIGH_ADC 51066  // 5V = 112psig = 772kPa
#define PRESSURE_SENSOR_CALIBRATION_LOW_KPA 101
#define PRESSURE_SENSOR_CALIBRATION_HIGH_KPA 583 // 772

#define FLOW_RATE_MIN_THRESHOLD 300
#define FLOW_RATE_MIN_TIME_MS_THRESHOLD 5000

#define DESIRED_TEMP 4.5
#define COMPRESSOR_UNDER_TEMP_CUTOFF 3.0
#define COMPRESSOR_RESUME_PID_CONTROL_TEMP 7.0
#define COMPRESSOR_PID_MAX_PCT 0.74

enum class CoolerSystemStatus {
    REQUIRES_RESET = 0,
    RESET          = 1,
    PRECHILL       = 2,
    PUMP_LOW       = 3,
    PUMP_MEDIUM    = 4,
    PUMP_HIGH      = 5
};

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
    uint8_t compressorPWM;
};

class CoolerSystem {
private:
    // inputs
    CalibratedADC switchADC, pressureSensor;
    uint8_t coolantLevelPin;
    FlowSensor flowSensor;
    NTC evaporatorInletNTC, evaporatorOutletNTC, condenserInletNTC, condenserOutletNTC, ambientNTC;

    // outputs
    PWMOutput compressorPWM, coolshirtPWM, chillerPumpPWM, systemEnableOutput;

    // internal state
    Bounce coolantLevelBounce;
    VoltageMonitor& voltageMonitor;
    byte _systemFault { (byte)SystemFault::SYSTEM_STARTUP };
    CoolerSystemStatus systemStatus { CoolerSystemStatus::REQUIRES_RESET };

    Metro pollTimer { Metro(100) };
    Metro displayInfoTimer { Metro(2000) };
    Metro msTick = { Metro(1) };
    uint32_t pumpStartTime = { 0 };
    CoolerSystemStatus switchPosition { CoolerSystemStatus::RESET };

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

    double compressorOutputValue { 0 };
    double compressorTempTarget { DESIRED_TEMP };
    bool undertempCutoff { false };
    const double Kp=2.67, Ki=2.4, Kd=5.8;
    PID compressorPID = {
        PID(
            &evaporatorOutletTemp, &compressorOutputValue, &compressorTempTarget,
            Kp, Ki, Kd, true, DIRECT
        )
    };

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
        uint8_t _coolantLevelPin,
        uint8_t _flowRatePin,
        uint8_t _pressureSensorPin,
        uint8_t _evaporatorInletNtcPin,
        uint8_t _evaporatorOutletNtcPin,
        uint8_t _condenserInletNtcPin,
        uint8_t _condenserOutletNtcPin,
        uint8_t _ambientNtcPin,
        uint8_t _compressorPin,
        uint8_t _chillerPumpPin,
        uint8_t _coolshirtPumpPin,
        uint8_t _systemEnablePin,
        VoltageMonitor &_voltageMonitor
    )
        : switchADC { CalibratedADC(_switchPin) }
        , pressureSensor { CalibratedADC(_pressureSensorPin) }
        , coolantLevelPin { _coolantLevelPin }
        , flowSensor { FlowSensor(_flowRatePin, FLOW_SENSOR_PULSES_PER_SECOND) }
        , evaporatorInletNTC { NTC(_evaporatorInletNtcPin, 6800) }
        , evaporatorOutletNTC { NTC(_evaporatorOutletNtcPin, 6800) }
        , condenserInletNTC { NTC(_condenserInletNtcPin, 15000) }
        , condenserOutletNTC { NTC(_condenserOutletNtcPin, 15000) }
        , ambientNTC { NTC(_ambientNtcPin, 6800) }
        , compressorPWM { PWMOutput(_compressorPin, true) }
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
