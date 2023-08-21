#include "constants.h"

#include "Arduino.h"
#include <Metro.h>
#include <DebugLog.h>
#include <FlexCAN.h>

#include "canlogger.h"
#include "clocktime.h"
#include "ledstatus.h"
#include "utils.h"
#include "coolersystem.h"


Metro statsTimer = Metro(10000);
Metro canBroadcastTimer = Metro(500);
Metro msTick = Metro(1);

static CAN_message_t coolerSystemMessage, rxMessage;

CoolerSystem cooler = CoolerSystem(
    EXTERNAL_ADC1, // switchPin
    EXTERNAL_ADC2, // coolantLevelPin,
    EXTERNAL_ADC3, // flowRatePin,
    EXTERNAL_ADC4, // pressureSensorPin,
    SPI_CS_TC1,    // _thermocoupleCSPin,
    EXTERNAL_PWM1, // compressorPin,
    EXTERNAL_PWM2, // chillerPumpPin,
    EXTERNAL_PWM3, // coolshirtPumpPin,
    EXTERNAL_PWM4  // systemEnablePin
);

unsigned long previousLoop, loopStart;

void setup()
{
    LOG_SET_LEVEL(DebugLogLevel::LVL_DEBUG);
    analogReadRes(10);
    cooler.setup();
    LEDStatus::setup();
    Serial.begin(115200);
    Can0.begin(500000);
    CANLogger::setup();
    LOG_INFO("System Boot OK");
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
    cooler.loop();
    // end tick functions

    if (canBroadcastTimer.check()) {
        cooler.getCANMessage(coolerSystemMessage);
        broadcastMessage(coolerSystemMessage);
    }

    // // read canbus data if message is available
    // if (Can0.read(rxMessage))
    // {
    //     processRXCANMessage();
    // }

    if (statsTimer.check())
    {
        CANLogger::logComment("loop_time=" + (String)loopTime + "uS, error_code=" + hexDump(LEDStatus::getError()));
        LOG_INFO("loop_time=" + (String)loopTime + "uS, error_code=" + hexDump(LEDStatus::getError()));
    }
}
