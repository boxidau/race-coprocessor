#include "constants.h"

#include "Arduino.h"
#include <Metro.h>
#include <SPI.h>
#include <DebugLog.h>
#include <FlexCAN.h>

#include "canlogger.h"
#include "thermocouple.h"
#include "clocktime.h"
#include "ledstatus.h"
#include "utils.h"
#include "analogsensors.h"
#include "dac.h"
#include "dio.h"


Metro statsTimer = Metro(10000);
Metro pollTimer = Metro(50);
Metro msTick = Metro(1);
Metro ledTimer = Metro(100);

static CAN_message_t thermocoupleMessage, analogMessage, dioMessage, ntcMessage, rxMessage;

const int adcSensorPins[4] = {EXTERNAL_ADC1, EXTERNAL_ADC2, EXTERNAL_ADC3, EXTERNAL_ADC4};
const int ntcSensorPins[2] = {EXTERNAL_NTC1, EXTERNAL_NTC2};
AnalogSensors analogSensors = AnalogSensors(adcSensorPins, 4, CANID_ANALOG);
AnalogSensors ntcSensors = AnalogSensors(ntcSensorPins, 2, CANID_NTC);

const int dioInputPins[4] = {DIGITAL_IO1, DIGITAL_IO2, DIGITAL_IO3, DIGITAL_IO4};
const int dioOutputPins[3] = {DIGITAL_IO5, DIGITAL_IO6, DIGITAL_IO7};
DIO dio = DIO(dioInputPins, 4, dioOutputPins, 3, CANID_DIO);

unsigned long previousLoop, loopStart;

void setup()
{
    analogReadRes(10);

    pinMode(EXTERNAL_PWM1, OUTPUT);
    pinMode(EXTERNAL_PWM2, OUTPUT);
    pinMode(EXTERNAL_DIGITAL_PIN, INPUT);
    pinMode(LED2, OUTPUT);

    Serial.begin(115200);
    SPI.begin();
    Can0.begin(500000);
    delay(500);
    LOG_INFO("System Boot");

    analogSensors.setup();
    ntcSensors.setup();

    LEDStatus::setup();
    Thermocouple::setup();
    DAC::setup(0, 200);
    dio.setup();
    LOG_INFO("Modules Initialized");

    CANLogger::setup();
    LOG_INFO("CAN Logger Initialized");
}

void broadcastMessage(CAN_message_t &message)
{
    Can0.write(message);
    CANLogger::logCANMessage(message, CAN_TX);
}

void processRXCANMessage()
{
    CANLogger::logCANMessage(rxMessage, CAN_RX);
}

void loop()
{
    loopStart = micros();
    unsigned long loopTime = loopStart - previousLoop;
    previousLoop = loopStart;

    // tick functions for all modules
    LEDStatus::loop();
    // end tick functions

    if (ledTimer.check()) {
        digitalWrite(LED2, !digitalRead(LED2));
    }

    // poll all data and write to TX_CAN/log
    if (pollTimer.check())
    {
        LOG_TRACE("Polling modules");
        // EGT poll and log
        Thermocouple::getCANMessage(thermocoupleMessage);
        LEDStatus::setError(EGT_ERROR, Thermocouple::getError() > 0);
        broadcastMessage(thermocoupleMessage);

        // analog sensor poll and log
        analogSensors.getCANMessage(analogMessage);
        broadcastMessage(analogMessage);

        // ntc sensor poll and log
        ntcSensors.getCANMessage(ntcMessage);
        broadcastMessage(ntcMessage);

        dio.getStatusCANMessage(dioMessage);
        broadcastMessage(dioMessage);

        // Logger error status
        LEDStatus::setError(LOGGER_ERROR, CANLogger::error);

        DAC::update(0);
    }


    // read canbus data if message is available
    if (Can0.read(rxMessage))
    {
        processRXCANMessage();
    }

    if (statsTimer.check())
    {
        CANLogger::logComment("loop_time=" + (String)loopTime + "uS, error_code=" + hexDump(LEDStatus::getError()));
        LOG_INFO("loop_time=" + (String)loopTime + "uS, error_code=" + hexDump(LEDStatus::getError()));
    }
}
