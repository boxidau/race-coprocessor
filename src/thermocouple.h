#include "Arduino.h"
#include "constants.h"
#include <SPI.h>

struct ThermocoupleMessage
{
    float temperature;
    int16_t scaledTemperature; // scale factor 2^-2
    float internalTemperature;
    int16_t scaledInternalTemperature; // scale factor 2^-4
    byte error;
};

class Thermocouple
{
public:
    static void setup();
    static void readData(const int chipSelectPin, ThermocoupleMessage &msg);
};

// MAX31855KASA RX byte map
// byte | bit | purpose
// ---------------------------
// 0    | 31  | tc temp sign
// 0    | 30  | tc temp MSB 2^10
// 0    | 29  | tc temp
// 0    | 28  | tc temp
// 0    | 27  | tc temp
// 0    | 26  | tc temp
// 0    | 25  | tc temp
// 0    | 24  | tc temp
// ---------------------------
// 1    | 23  | tc temp
// 1    | 22  | tc temp
// 1    | 21  | tc temp
// 1    | 20  | tc temp 2^0
// 1    | 19  | tc temp 2^-1
// 1    | 18  | tc temp 2^-2
// 1    | 17  | reserved
// 1    | 16  | fault
// ---------------------------
// 2    | 15  | int temp sign
// 2    | 14  | int temp MSB 2^6
// 2    | 13  | int temp
// 2    | 12  | int temp
// 2    | 11  | int temp
// 2    | 10  | int temp
// 2    |  9  | int temp
// 2    |  8  | int temp
// ---------------------------
// 3    |  7  | int temp 2^-1
// 3    |  6  | int temp 2^-2
// 3    |  5  | int temp 2^-3
// 3    |  4  | int temp 2^-4
// 3    |  3  | reserved
// 3    |  2  | SCV (short to vcc)
// 3    |  1  | SCG (short to gnd)
// 3    |  0  | OC (open circuit)
// ---------------------------
