#include "Arduino.h"
#include <DebugLog.h>
#include <SPI.h>
#include <Bounce.h>
#include <PID_v1.h>
#include <Metro.h>
#include <FlexCAN.h>
#include "thermocouple.h"

// 7.5 pulses per second == flow rate in LPM
// therefore 1000000uS / 7.5 * 1000 is the numerator for the pulseInterval (in uS)
// flow is expessed in mL/min so we can use ints instead of floats
#define MLPM_MAGIC_NUMBER 133333333

#define UNDERPRESSURE_THRESHOLD_KPA 7
#define OVERPRESSURE_THRESHOLD_KPA 300
#define PRESSURE_SENSOR_CALIBRATION_LOW_ADC 102 // 0.5V = 0psig = 101kPa
#define PRESSURE_SENSOR_CALIBRAION_HIGH_ADC 922 // 4.5V = 100psig = 689kPa
#define PRESSURE_SENSOR_CALIBRATION_LOW_KPA 101
#define PRESSURE_SENSOR_CALIBRATION_HIGH_KPA 689

#define FLOW_RATE_MIN_THRESHOLD 300
#define FLOW_RATE_MIN_TIME_MS_THRESHOLD 5000

#define DESIRED_TEMP 5
#define COMPRESSOR_UNDER_TEMP_CUTOFF 3
#define COMPRESSOR_RESUME_PID_CONTROL_TEMP 7


enum class CoolerSwitchPosition {
    REQUIRES_RESET = 0,
    RESET          = 1,
    PRECHILL       = 2,
    PUMP_LOW       = 3,
    PUMP_MEDIUM    = 4,
    PUMP_HIGH      = 5
};

constexpr const char* CoolerSwitchPositionToString(CoolerSwitchPosition csp)
{
    switch (csp)
    {
        case CoolerSwitchPosition::REQUIRES_RESET: return "REQUIRES_RESET";
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
    THERMOCOUPLE_ERROR = 4,
    SWITCH_ADC_OUT_OF_BOUNDS = 8,
    SYSTEM_STARTUP = 16
};

constexpr const char* SystemFaultToString(SystemFault sf)
{
    switch (sf)
    {
        case SystemFault::SYSTEM_OK: return "SYSTEM_OK";
        case SystemFault::LOW_COOLANT: return "LOW_COOLANT";
        case SystemFault::FLOW_RATE_LOW: return "FLOW_RATE_LOW";
        case SystemFault::THERMOCOUPLE_ERROR: return "THERMOCOUPLE_ERROR";
        case SystemFault::SWITCH_ADC_OUT_OF_BOUNDS: return "SWITCH_ADC_OUT_OF_BOUNDS";
        case SystemFault::SYSTEM_STARTUP: return "SYSTEM_STARTUP";
        default: return "UNKNOWN";
    }
}

class CoolerSystem {
private:
    // inputs
    uint8_t switchPin, coolantLevelPin, flowRatePin, pressureSensorPin, thermocoupleCSPin;
    // outputs
    uint8_t compressorPin, chillerPumpPin, coolshirtPumpPin, systemEnablePin;

    // internal state
    bool systemRequiresReset = { true };
    CoolerSwitchPosition switchPosition = { CoolerSwitchPosition::REQUIRES_RESET };
    bool coolantLevel = { false };
    uint16_t systemPressure = 0;
    ThermocoupleMessage evaporatorTemp;
    uint16_t flowRate = { 0 };  // mL / min

    // output states
    // for logging/canbus output
    // systemEnable and chillerPump can just digitalRead their own state
    uint16_t coolshirtPumpValue = { 0 };
    uint16_t compressorValue = { 0 };

    double compressorInputTemp = { 0 };
    double compressorOutputValue = { 0 };
    double compressorTempTarget = { DESIRED_TEMP };
    bool undertempCutoff = { false };
    const double Kp=2, Ki=5, Kd=1;
    PID compressorPID = {
        PID(
            &compressorInputTemp, &compressorOutputValue, &compressorTempTarget,
            Kp, Ki, Kd, REVERSE
        )
    };

    // getters
    // will return REQUIRES_RESET until the switch has visited the RESET position at least once
    // in a panic condition the need to re-visit the reset is enabled
    CoolerSwitchPosition _getSwitchPosition();

    // pollers/updaters
    void _pollSystemPressure();
    void _pollSwitchPosition();
    void _pollCoolantLevel();
    void _pollFlowRate();

    // executors
    void runChillerPump();
    void runCompressor();
    void runCoolshirtPump();
    void panic(SystemFault fault);


public:
    CoolerSystem(
        uint8_t _switchPin,
        uint8_t _coolantLevelPin,
        uint8_t _flowRatePin,
        uint8_t _pressureSensorPin,
        uint8_t _thermocoupleCSPin,
        uint8_t _compressorPin,
        uint8_t _chillerPumpPin,
        uint8_t _coolshirtPumpPin,
        uint8_t _systemEnablePin
    )
        : switchPin { _switchPin }
        , coolantLevelPin { _coolantLevelPin }
        , flowRatePin { _flowRatePin }
        , pressureSensorPin { _pressureSensorPin }
        , thermocoupleCSPin { _thermocoupleCSPin }
        , compressorPin { _compressorPin }
        , chillerPumpPin { _chillerPumpPin }
        , coolshirtPumpPin { _coolshirtPumpPin }
        , systemEnablePin { _systemEnablePin }

    {};
    void setup();
    void loop();
    void getCANMessage(CAN_message_t &msg);
    SystemFault systemFault = { SystemFault::SYSTEM_STARTUP };
};