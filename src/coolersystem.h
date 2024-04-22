#pragma once

#include "Arduino.h"

#include <DebugLog.h>
#include <SPI.h>
#include <Bounce.h>
#include <PID_v1.h>
#include <Metro.h>
#include <FlexCAN.h>
#include <FixedPointBiquad.h>

#include "constants.h"
#include "pwm.h"
#include "ntc.h"
#include "calibratedadc.h"
#include "switchadc.h"
#include "voltagemonitor.h"
#include "flowsensor.h"
#include "compressorfault.h"
#include "samplelogger.h"
#include "stringformat.h"
#include "timer.h"
#include "analyzenotefrequency.h"

#define OVERPRESSURE_THRESHOLD_KPA 250 // operating pressure ~170 - 200kPa
#define PRESSURE_SENSOR_CALIBRATION_LOW_ADC 5900 // 0.5V = 0psig = 101kPa
#define PRESSURE_SENSOR_CALIBRATION_HIGH_ADC 51063  // 4.5V = 100psig = 791kPa
#define PRESSURE_SENSOR_CALIBRATION_LOW_KPA 101
#define PRESSURE_SENSOR_CALIBRATION_HIGH_KPA 791

#define CURRENT_SENSOR_CALIBRATION_LOW_ADC 5965 // 0.5V = 0A
#define CURRENT_SENSOR_CALIBRATION_HIGH_ADC 51351 // 4.5V = 50A
#define CURRENT_SENSOR_CALIBRATION_LOW_AMPS 0
#define CURRENT_SENSOR_CALIBRATION_HIGH_AMPS 50000

#define FLOW_SENSOR_PULSES_PER_SECOND 7.5
#define FLOW_RATE_MIN_THRESHOLD 1.0 // Lpm
#define FLOW_RATE_STARTUP_TIME 5000 // ms allowed until the flow rate must be above threshold
#define FLOW_RATE_MISSING_PULSE_TIME 1000 // ms allowed since the last pulse seen
#define SPECIFIC_HEAT 4033 // J/kgK of chiller fluid (90% water / 10% IPA @ 3C)
#define FLOW_RATE_TARGET 3.5 // Lpm
#define CHILLER_PUMP_DEFAULT_SPEED 0.6
#define CHILLER_PUMP_PID_KP 0.05
#define CHILLER_PUMP_PID_KI 0.25 // can try as high as 0.5 if stability is good
#define CHILLER_PUMP_PID_KD 0
#define CHILLER_PUMP_MIN_SPEED 0
#define CHILLER_PUMP_MAX_SPEED 0.8

#define COMPRESSOR_UNDER_TEMP_CUTOFF_HIGH 3.5
#define COMPRESSOR_RESTART_TEMP_HIGH 6.5
#define COMPRESSOR_UNDER_TEMP_CUTOFF_MED 8.5
#define COMPRESSOR_RESTART_TEMP_MED 11.5
#define COMPRESSOR_UNDER_TEMP_CUTOFF_LOW 13.5
#define COMPRESSOR_RESTART_TEMP_LOW 16.5
#define EVAPORATOR_OUTLET_PANIC_TEMPERATURE 0.0

// valid range of compressor speed output is 4.5V = 50%, 9V = 100%
#define COMPRESSOR_MIN_SPEED_RATIO 0.5
#define COMPRESSOR_MAX_SPEED_RATIO 1.0
#define COMPRESSOR_DEFAULT_SPEED 0.75
#define COMPRESSOR_SPEED_RATIO_TO_ANALOG (9 / (3.3 * 3.717) * ADC_MAX)
#define COMPRESSOR_MIN_COOLDOWN_MS 60000
#define COMPRESSOR_PID_KP 0.5
#define COMPRESSOR_PID_KI 0
#define COMPRESSOR_PID_KD 0

// max flush time with pumps running, don't want to let them run dry for long
#define FLUSH_TIMEOUT_MS 30000

// time to acquire data and stabilize before doing anything
#define STARTUP_STABILIZATION_SAMPLES 100

#define UPDATE_STATE_TIMER_MS 100
#define DATA_LOG_INTERVAL_MS 100
#define DISPLAY_INFO_MS 2000

enum class CoolerSystemStatus {
    STARTUP         = 0,
    REQUIRES_RESET  = 1,
    RESET           = 2,
    PRECHILL        = 3,
    PUMP_LOW        = 4,
    PUMP_MEDIUM     = 5,
    PUMP_HIGH       = 6,
    FLUSH           = 7
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
        case CoolerSystemStatus::FLUSH: return "FLUSH";
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
    CoolerSystemStatus systemStatus;
    CompressorFaultCode compressorFaultCode;
    byte fault;
    bool coolantLevel;
    uint16_t systemPressure;
    uint16_t flowRate;
    float evaporatorInletTemp;
    float evaporatorOutletTemp;
    float condenserInletTemp;
    float condenserOutletTemp;
    float ambientTemp;
    float compressorSpeed;
    float compressorCurrent;
    float compressorFrequency;
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
    PrecisionNTC evaporatorInletNTC, evaporatorOutletNTC;
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

    uint32_t sampleCounter { 0 };
    uint32_t loggedSampleCounter { 0 };
    MetroTimer msTick = { MetroTimer(1) };
    MetroTimer updateStateTimer = { MetroTimer(UPDATE_STATE_TIMER_MS) };
    MetroTimer dataLogTimer = { MetroTimer(DATA_LOG_INTERVAL_MS) };
    MetroTimer displayInfoTimer = { MetroTimer(DISPLAY_INFO_MS) };
    uint32_t pumpStartTime = { 0 };
    bool shouldFlush { false };
    FlexCAN& CANBus;

#if NTC_DEBUG || FLOW_DEBUG
    SampleLogger sampleLogger;
#endif

    // sensor values
    CompressorFaultCode compressorFaultCode { CompressorFaultCode::OK };
    float flowRate { 0 };
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
    float evaporatorInletTemp { -100.0 };
    float compressorSpeed { 0 };
    float compressorTempTarget { 5 };
    bool undertempCutoff { false };
    PID compressorPID {
        &evaporatorInletTemp,
        &compressorSpeed,
        &compressorTempTarget,
        COMPRESSOR_PID_KP,
        COMPRESSOR_PID_KI,
        COMPRESSOR_PID_KD,
        P_ON_M,
        REVERSE
    };

    float instantaneousFlowRate { 0 };
    float chillerPumpSpeed { CHILLER_PUMP_DEFAULT_SPEED };
    float flowRateTarget { FLOW_RATE_TARGET };
    PID chillerPumpPID {
        &instantaneousFlowRate,
        &chillerPumpSpeed,
        &flowRateTarget,
        CHILLER_PUMP_PID_KP,
        CHILLER_PUMP_PID_KI,
        CHILLER_PUMP_PID_KD,
        P_ON_E,
        DIRECT
    };

    // Biquad IIR filtering for compressor current frequency measurement
    FixedPointBiquad biquad;
    AnalyzeNoteFrequency analyzeNoteFrequency;
    float compressorFrequency { 0 };
    float compressorFrequencyProbability { 0 };
    float compressorCurrentBiquadOutput;

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
    void updateCoolerData();
    void updateFaults();
    void updateState();
    void updateOutputs();
    void displayInfo();
    void getLogMessage(StringFormatCSV& format);
    void getCANMessage(CAN_message_t &msg);
    void logData();
    const char* getLogHeader();

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
        uint8_t sysp3v3ADCNum,
        FlexCAN& canbus
    )
        : switchADC(_switchPin, _switchADCNum)
        , pressureSensor(_pressureSensorPin, _pressureSensorADCNum)
        , currentSensor(_currentSensorPin, _currentSensorADCNum)
        , compressorFault(_compressorLDRPin)
        , coolantLevelPin { _coolantLevelPin }
        , compressorSpeedPin { _compressorSpeedPin }
        , flowSensor(_flowRatePin, FLOW_SENSOR_PULSES_PER_SECOND, FLOW_RATE_MISSING_PULSE_TIME)
        , evaporatorInletNTC(_evaporatorInletNtcPin, _ntcADCNum, 15000, TDK_THERMISTOR_1_STEINHART_A, TDK_THERMISTOR_1_STEINHART_B, TDK_THERMISTOR_1_STEINHART_C)
        , evaporatorOutletNTC(_evaporatorOutletNtcPin, _ntcADCNum, 15000, TDK_THERMISTOR_2_STEINHART_A, TDK_THERMISTOR_2_STEINHART_B, TDK_THERMISTOR_2_STEINHART_C)
        , condenserInletNTC(_condenserInletNtcPin, _ntcADCNum, 6800, TE_THERMISTOR_STEINHART_A, TE_THERMISTOR_STEINHART_B, TE_THERMISTOR_STEINHART_C)
        , condenserOutletNTC(_condenserOutletNtcPin, _ntcADCNum, 6800, TE_THERMISTOR_STEINHART_A, TE_THERMISTOR_STEINHART_B, TE_THERMISTOR_STEINHART_C)
        , ambientNTC(_ambientNtcPin, _ntcADCNum, 6800, TE_THERMISTOR_STEINHART_A, TE_THERMISTOR_STEINHART_B, TE_THERMISTOR_STEINHART_C)
        , coolshirtPWM(_coolshirtPumpPin)
        , chillerPumpPWM(_chillerPumpPin)
        , systemEnableOutput(_systemEnablePin)
        , coolantLevelBounce(_coolantLevelPin, 10)
        , voltageMonitor(sys12vPin, sys12vADCNum, sys5vPin, sys5vADCNum, sys3v3Pin, sys3v3ADCNum, sysp3v3Pin, sysp3v3ADCNum)
        , CANBus(canbus)
        , biquad(bq_type_bandpass, 70.0 / 1000.0, 3, 0) // center freq / sample rate; Q; gain (dB)
        , analyzeNoteFrequency(1000, 0.7) // sample rate (Hz); allowed uncertainty in detection
    {
    };

    void setupIO();
    void setupLogging();
    void loop();
    void getSystemData(CoolerSystemData &data);
    uint32_t lastFlowPulseMicros();
    void setCompressorSpeedPercent(uint32_t percent);
    void setCompressorSpeedPercentOffset(int32_t offset);
    void toggleFlush();
};
