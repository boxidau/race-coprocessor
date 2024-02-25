#include "constants.h"

#include "Arduino.h"
#include <Metro.h>
#include <DebugLog.h>
#include <FlexCAN.h>
#include <ADC.h>

#include "canlogger.h"
#include "clocktime.h"
#include "utils.h"
#include "coolersystem.h"
#include "ui.h"
#include "voltagemonitor.h"
#include "singletonadc.h"
#include "looptimer.h"
#include "stringformat.h"

#define HSR_LOGGER

Metro statsTimer = Metro(10000, 1);
Metro canBroadcastTimer = Metro(100, 1);
LoopTimer loopTimer = LoopTimer();

FlexCAN CANbus = FlexCAN(500000, 0, true, true);

CAN_message_t coolerSystemMessage, rxMessage;

VoltageMonitor voltageMonitor = VoltageMonitor(
    ADC_SYSTEM_12V,
    ADC_SYSTEM_12V_ADC_NUM,
    ADC_SYSTEM_5V,
    ADC_SYSTEM_5V_ADC_NUM,
    ADC_SYSTEM_3V3,
    ADC_SYSTEM_3V3_ADC_NUM,
    ADC_SPARE_5V, // P3V3 using 5V spare ADC
    ADC_SPARE_5V_ADC_NUM
);

CoolerSystem cooler = CoolerSystem(
    ADC_MAIN_SWITCH, // switchPin
    ADC_MAIN_SWITCH_ADC_NUM, // switchADCNum
    COOLANT_SWITCH, // coolantLevelPin,
    FLOW_SENSOR, // flowRatePin,
    ADC_PRESSURE_SENSOR, // pressureSensorPin,
    ADC_PRESSURE_SENSOR_ADC_NUM, // pressureSensorADCNum,
    ADC_COMPRESSOR_CURRENT, // currentSensorPin,
    ADC_COMPRESSOR_CURRENT_ADC_NUM, // currentSensorADCNum,
    ADC_COMPRESSOR_LDR, // compressorLDRPin,
    NTC_EVAPORATOR_1, // _ntc1Pin,
    NTC_EVAPORATOR_2, // _ntc2Pin,
    NTC_CONDENSER_1, // condenser inlet
    NTC_CONDENSER_2, // condenser outlet
    NTC_AMBIENT,
    NTC_ADC_NUM,
    DAC_COMPRESSOR_SPEED, // compressorSpeedPin,
    PWM1, // chillerPumpPin,
    PWM2, // coolshirtPumpPin,
    PWM3, // systemEnablePin
    voltageMonitor
);

CoolerUI ui = CoolerUI(cooler, SPI_DISPLAY_CS, UI_BUTTON);

void setup()
{
    // LOG_SET_LEVEL(DebugLogLevel::LVL_DEBUG);
    // initialize pin inputs/outputs first thing so they stabilize
    cooler.setup();
    ui.setup();

    // Set up and calibrate ADCs, see https://forum.pjrc.com/index.php?threads/adc-library-with-support-for-teensy-4-3-x-and-lc.25532/
    // adc0 is for NTCs, adc1 everything else
    ADC *adc = SingletonADC::getADC();
    adc->adc0->setResolution(16);
    adc->adc0->setReference(ADC_REFERENCE::REF_EXT);
    adc->adc0->setAveraging(0);
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);

    adc->adc1->setResolution(16);
    adc->adc1->setReference(ADC_REFERENCE::REF_EXT);
    adc->adc1->setAveraging(0);
    adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
    adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);

    analogWriteRes(16);
    adc->adc0->recalibrate();
    adc->adc1->recalibrate();

    Serial.begin(115200);
    ClockTime::setup();
    CANbus.begin();
    CANLogger::setup();

    LOG_INFO("System Boot OK");
}

void broadcastMessage(CAN_message_t &message)
{
    CANbus.write(message);
    CANLogger::logCANMessage(message, CAN_TX);
}

void processRXCANMessage()
{
    CANLogger::logCANMessage(rxMessage, CAN_RX);
}

static uint32_t slowLoopTime = 0;
static bool coolerHasStarted = false;

void loop()
{
    uint32_t loopTime = loopTimer.start();

    // tick functions for all modules
    cooler.loop();
    ui.loop();
    // end tick functions

    bool logFirstData = false;
    if (cooler.hasStarted() && !coolerHasStarted) {
        coolerHasStarted = true;
        canBroadcastTimer.reset();
        logFirstData = true;
    }

    if (coolerHasStarted && canBroadcastTimer.check() || logFirstData) {
        char message[256];
        StringFormatCSV format(message, sizeof(message));
        cooler.getLogMessage(format);
        format.formatUnsignedInt(slowLoopTime);
        CANLogger::logMessage(format);
        slowLoopTime = 0;
        //cooler.getCANMessage(coolerSystemMessage);
        //broadcastMessage(coolerSystemMessage);
    }

    // // read canbus data if message is available
    // if (Can0.read(rxMessage))
    // {
    //     processRXCANMessage();
    // }

    if (statsTimer.check())
    {
        //CANLogger::logComment("loop_time=" + (String)loopTime + "uS");
    }

    if (loopTime > 1000) {
        LOG_INFO("SLOW LOOP:", loopTime, "us");
        slowLoopTime += loopTime;
    }
}
