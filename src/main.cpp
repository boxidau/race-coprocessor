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

#define HSR_LOGGER

Metro statsTimer = Metro(10000);
Metro canBroadcastTimer = Metro(100);

FlexCAN CANbus = FlexCAN(500000);

static CAN_message_t coolerSystemMessage, rxMessage;

CoolerSystem cooler = CoolerSystem(
    EXTERNAL_ADC1, // switchPin
    EXTERNAL_ADC3, // coolantLevelPin,
    EXTERNAL_ADC2, // flowRatePin,
    EXTERNAL_ADC4, // pressureSensorPin,
    EXTERNAL_NTC1, // _ntc1Pin,
    EXTERNAL_NTC2, // _ntc2Pin,
    EXTERNAL_PWM2, // compressorPin,
    EXTERNAL_PWM3, // chillerPumpPin,
    EXTERNAL_PWM4, // coolshirtPumpPin,
    EXTERNAL_PWM1,  // systemEnablePin
    LED3 // pulseFlowPin
);

LEDStatus ledStatus = LEDStatus(LED1, LED2);

unsigned long previousLoop, loopStart;

void setup()
{
    // LOG_SET_LEVEL(DebugLogLevel::LVL_DEBUG);
    analogReadRes(16);
    analogWriteRes(16);
    cooler.setup();
    ledStatus.setup();
    CANbus.begin();
    Serial.begin(115200);
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

void loop()
{
    loopStart = micros();
    unsigned long loopTime = loopStart - previousLoop;
    previousLoop = loopStart;

    // tick functions for all modules
    cooler.loop();
    ledStatus.error = cooler.systemFault();
    ledStatus.loop();
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
        CANLogger::logComment("loop_time=" + (String)loopTime + "uS, error_code=" + hexDump(ledStatus.error));
    }

    if (loopTime > 1000) {
        LOG_INFO("loop_time=" + (String)loopTime + "uS, error_code=" + hexDump(ledStatus.error));
    }
}
