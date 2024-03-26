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
#include "singletonadc.h"
#include "looptimer.h"
#include "stringformat.h"

LoopTimer loopTimer = LoopTimer();

FlexCAN CANbus = FlexCAN(500000, 0, true, true);

CAN_message_t rxMessage;

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
    ADC_SYSTEM_12V,
    ADC_SYSTEM_12V_ADC_NUM,
    ADC_SYSTEM_5V,
    ADC_SYSTEM_5V_ADC_NUM,
    ADC_SYSTEM_3V3,
    ADC_SYSTEM_3V3_ADC_NUM,
    ADC_SPARE_5V, // P3V3 using 5V spare ADC
    ADC_SPARE_5V_ADC_NUM,
    CANbus
);

CoolerUI ui = CoolerUI(cooler, SPI_DISPLAY_CS, UI_BUTTON);

uint32_t mapInputToCompressorSpeed(char input) {
    switch (input) {
        case '1':
            return 50;
        case '2':
            return 60;
        case '3':
            return 70;
        case '4':
            return 80;
        case '5':
            return 90;
        case '6':
        default:
            return 100;
    }
}

void setup()
{
    LOG_SET_LEVEL(DebugLogLevel::LVL_DEBUG);

    // initialize pin inputs/outputs first thing so they stabilize
    cooler.setupIO();
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
    cooler.setupLogging();

    LOG_INFO("System Boot OK");
    LOG_INFO("Type 1, 2, 3, 4, 5, 6 = 50%, 60%, 70%, 80%, 90%, 100% compressor speed, f = flush coolant");
}

void processRXCANMessage()
{
    //CANLogger::logCANMessage(rxMessage, CAN_RX);
}

void loop()
{
    uint32_t loopTime = loopTimer.start();

    if (Serial.available()) {
        char input = Serial.read();
        LOG_INFO("Received input", input);
        if (input >= '1' && input <= '6') {
            cooler.setCompressorSpeedPercent(mapInputToCompressorSpeed(input));
        } else if (input == 'f') {
            cooler.toggleFlush();
        }
    }

    // tick functions for all modules
    cooler.loop();
    ui.loop();
    // end tick functions

    // // read canbus data if message is available
    // if (Can0.read(rxMessage))
    // {
    //     processRXCANMessage();
    // }

    if (loopTime > 1000) {
        LOG_INFO("SLOW LOOP:", loopTime, "us");
    }
}
