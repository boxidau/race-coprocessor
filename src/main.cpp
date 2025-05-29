#include "constants.h"

#include "Arduino.h"
#include <Metro.h>
#include <DebugLog.h>
#include <FlexCAN.h>
#include <ADC.h>

#include "watchdog.h"
#include "canlogger.h"
#include "clocktime.h"
#include "utils.h"
#include "coolersystem.h"
#include "ui.h"
#include "singletonadc.h"
#include "looptimer.h"
#include "stringformat.h"

LoopTimer loopTimer = LoopTimer();

// try higher baud, 1M. 100k/250k also valid
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
    NTC_EVAPORATOR_DIFF_1, // _ntc1Pin,
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

CoolerUI ui = CoolerUI(cooler, SPI_DISPLAY_CS, UI_BUTTON, PWM4);

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
    adc->adc0->setAveraging(1);
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);

    adc->adc1->setResolution(16);
    adc->adc1->setReference(ADC_REFERENCE::REF_EXT);
    adc->adc1->setAveraging(1);
    adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
    adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);

    analogWriteRes(16);
    adc->adc0->recalibrate();
    adc->adc1->recalibrate();

#if RC_DEBUG || LOG_SLOW_LOOPS
    Serial.begin(115200);
#endif
    ClockTime::setup();
    CANbus.begin();
    cooler.setupLogging();

    LOG_INFO("System Boot OK");
    LOG_INFO("Input 0 - 7 for compressor speed (0 = off, 1 = 50%, 7 = 100%), +/- = increment compressor speed 0.1%, c = resume automatic compressor control, f = flush coolant, r = toggle reset, p = toggle prechill");
}

void loop()
{
#if LOG_SLOW_LOOPS
    uint32_t loopTime = loopTimer.start();
#endif

#if RC_DEBUG
    if (Serial.available()) {
        char input = Serial.read();
        LOG_INFO("Received input", input);
        if (input >= '0' && input <= '7') {
            cooler.setCompressorSpeed(input - '0');
        } else if (input == '+' || input == '=') {
            cooler.setCompressorSpeedPercentOffset(1);
        } else if (input == '-') {
            cooler.setCompressorSpeedPercentOffset(-1);
        } else if (input == 'c') {
            cooler.resumeCompressorControl();
        } else if (input == 'f') {
            cooler.toggleFlush();
        } else if (input == 'p') {
            cooler.toggleSwitchPosition(CoolerSwitchPosition::PRECHILL);
        } else if (input == 'r') {
            cooler.toggleSwitchPosition(CoolerSwitchPosition::RESET);
        }
    }
#endif

    watchdog_ensure_init();

    // tick functions for all modules
    cooler.loop();
    ui.loop();
    // end tick functions

    // // read canbus data if message is available
    if (CANbus.available()) {
        CAN_message_t canMessage;
        canMessage.timeout = 0;

        uint32_t m = micros();
        int read = CANbus.read(canMessage);
        if (read && canMessage.id == CANID_RCP) {
            uint8_t lapCount = canMessage.buf[0];
            // log lap count
            cooler.setLapCount(lapCount);
            if (lapCount != 0) {
                // disable LEDs to save power
                ui.disableBoardLEDs();
            }
        } else {
            LOG_INFO("Received unknown CAN message id", canMessage.id);
        }
        LOG_INFO("CANBus read took", micros() - m, "us");
    }

#if LOG_SLOW_LOOPS
    static uint32_t loopMax = 0;
    if (loopTime > loopMax) {
        Serial.printf("[%u ms] New slowest loop: %u ms\n", millis(), loopTime);
        loopMax = loopTime;
    }
    if (loopTime > 800) {
        Serial.printf("[%u ms] Slow loop: %u ms\n", millis(), loopTime);
    }
#endif
}
