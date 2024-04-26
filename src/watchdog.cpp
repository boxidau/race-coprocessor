#include "watchdog.h"
#include <Arduino.h>

// 2 C/s drop in coolant temp if the evaporator is cooling at 700W
#define WATCHDOG_MILLISECONDS 200

static bool watchdogInited;

void watchdog_ensure_init() {
    if (watchdogInited) {
        return;
    }

    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    WDOG_TOVALH = WATCHDOG_MILLISECONDS >> 16;
    WDOG_TOVALL = WATCHDOG_MILLISECONDS & 0xFFFF;
    WDOG_PRESC = 0;
    WDOG_STCTRLH = WDOG_STCTRLH_WDOGEN;
    for (int i = 0; i < 8192; i++) {
        asm("nop");
    }

    watchdogInited = true;
}

void watchdog_reset() {
    noInterrupts();
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    interrupts();
}
