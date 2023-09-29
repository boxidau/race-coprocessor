#include "Arduino.h"
#include <DebugLog.h>
#include <SPI.h>
#include <ABounce.h>
#include <PID_v1.h>
#include <Metro.h>
#include <FlexCAN.h>

#include "ntc.h"
#include "constants.h"
#include "CoolerLog.h"

#define HSR_LOGGER true

// 7.5 pulses per second == flow rate in LPM
// therefore 1000000uS / 7.5 * 1000 is the numerator for the pulseInterval (in uS)
// flow is expessed in mL/min so we can use ints instead of floats
#define MLPM_MAGIC_NUMBER 133333333

#define OVERPRESSURE_THRESHOLD_KPA 170
#define PRESSURE_SENSOR_CALIBRATION_LOW_ADC 2112 // 0.5V = 0psig = 101kPa
#define PRESSURE_SENSOR_CALIBRAION_HIGH_ADC 20930  // 5V = 112psig = 772kPa
#define PRESSURE_SENSOR_CALIBRATION_LOW_KPA 101
#define PRESSURE_SENSOR_CALIBRATION_HIGH_KPA 772

#define FLOW_RATE_MIN_THRESHOLD 300
#define FLOW_RATE_MIN_TIME_MS_THRESHOLD 5000

#define DESIRED_TEMP 4.5
#define COMPRESSOR_UNDER_TEMP_CUTOFF 3.0
#define COMPRESSOR_RESUME_PID_CONTROL_TEMP 7.0


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
    SYSTEM_STARTUP = 128,
};

constexpr const char* SystemFaultToString(SystemFault sf)
{
    switch (sf)
    {
        case SystemFault::SYSTEM_OK: return "SYSTEM_OK";
        case SystemFault::LOW_COOLANT: return "LOW_COOLANT";
        case SystemFault::FLOW_RATE_LOW: return "FLOW_RATE_LOW";
        case SystemFault::SYSTEM_OVER_PRESSURE: return "SYSTEM_OVER_PRESSURE";
        case SystemFault::SYSTEM_STARTUP: return "SYSTEM_STATUP";
        default: return "UNKNOWN";
    }
}

class CoolerSystem {
private:
    // inputs
    uint8_t switchPin, coolantLevelPin, flowRatePin, pressureSensorPin;

    // ntc inputs
    NTC inletNTC, outletNTC;

    // outputs
    uint8_t compressorPin, chillerPumpPin, coolshirtPumpPin, systemEnablePin, flowPulsePin;

    // internal state
    byte _systemFault { (byte)SystemFault::SYSTEM_STARTUP };
    CoolerSystemStatus systemStatus { CoolerSystemStatus::REQUIRES_RESET };
    bool coolantLevel { false };
    uint16_t systemPressure { 0 };
    double chillerInletTemp { -100.0 };
    double chillerOutletTemp { -100.0 };
    uint16_t flowRate { 0 };  // mL / min

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
            &chillerOutletTemp, &compressorOutputValue, &compressorTempTarget,
            Kp, Ki, Kd, DIRECT
        )
    };

    // getters
    // will return REQUIRES_RESET until the switch has visited the RESET position at least once
    // in a panic condition the need to re-visit the reset is enabled
    CoolerSystemStatus _getSwitchPosition();

    // pollers/updaters
    void _pollSystemPressure();
    void _pollSystemStatus();
    void _pollCoolantLevel();
    void _pollFlowRate();
    void _pollNTCSensors();

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
        uint8_t _inletNtcPin,
        uint8_t _outletNtcPin,
        uint8_t _compressorPin,
        uint8_t _chillerPumpPin,
        uint8_t _coolshirtPumpPin,
        uint8_t _systemEnablePin,
        uint8_t _flowPulsePin
    )
        : switchPin { _switchPin }
        , coolantLevelPin { _coolantLevelPin }
        , flowRatePin { _flowRatePin }
        , pressureSensorPin { _pressureSensorPin }
        , inletNTC { NTC(_inletNtcPin) }
        , outletNTC { NTC(_outletNtcPin) }
        , compressorPin { _compressorPin }
        , chillerPumpPin { _chillerPumpPin }
        , coolshirtPumpPin { _coolshirtPumpPin }
        , systemEnablePin { _systemEnablePin }
        , flowPulsePin { _flowPulsePin }

    {};
    void setup();
    void loop();
    void getCANMessage(CAN_message_t &msg);
    byte systemFault();
};