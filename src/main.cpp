#include "constants.h"

#include "Arduino.h"
#include <Metro.h>
#include <DebugLog.h>
#include <FlexCAN.h>

#include "canlogger.h"
#include "clocktime.h"
#include "utils.h"
#include "coolersystem.h"
#include "ui.h"
#include "voltagemonitor.h"

#define HSR_LOGGER

Metro statsTimer = Metro(10000);
Metro canBroadcastTimer = Metro(100);

FlexCAN CANbus = FlexCAN(500000, 0, true, true);

CAN_message_t coolerSystemMessage, rxMessage;

VoltageMonitor voltageMonitor = VoltageMonitor(
    ADC_SYSTEM_12V,
    ADC_SYSTEM_5V,
    ADC_SYSTEM_3V3
);

CoolerSystem cooler = CoolerSystem(
    ADC_MAIN_SWITCH, // switchPin
    COOLANT_SWITCH, // coolantLevelPin,
    FLOW_SENSOR, // flowRatePin,
    ADC_PRESSURE_SENSOR, // pressureSensorPin,
    NTC_EVAPORATOR_1, // _ntc1Pin,
    NTC_EVAPORATOR_2, // _ntc2Pin,
    NTC_CONDENSER_1, // condenser inlet
    NTC_CONDENSER_2, // condenser outlet
    NTC_AMBIENT,
    DAC_COMPRESSOR_SPEED, // compressorPin,
    PWM1, // chillerPumpPin,
    PWM2, // coolshirtPumpPin,
    PWM3,  // systemEnablePin
    voltageMonitor
);

CoolerUI ui = CoolerUI(cooler, SPI_DISPLAY_CS, UI_BUTTON);

unsigned long previousLoop, loopStart;

void setup()
{
    // LOG_SET_LEVEL(DebugLogLevel::LVL_DEBUG);
    analogReadRes(16);
    analogWriteRes(16);
    cooler.setup();
    CANbus.begin();
    Serial.begin(115200);
    ClockTime::setup();
    CANLogger::setup();
    ui.setup();
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
    ui.loop();
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
        CANLogger::logComment("loop_time=" + (String)loopTime + "uS");
    }

    if (loopTime > 2000) {
        LOG_INFO("SLOW LOOP loop_time=" + (String)loopTime + "uS");
    }
}
