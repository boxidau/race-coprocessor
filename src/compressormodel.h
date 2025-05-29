#if 0
#include <Arduino.h>

enum class CompressorSpeed {
    OFF     = 0,
    SPEED_1 = 1,
    SPEED_2 = 2,
    SPEED_3 = 3,
    SPEED_4 = 4,
    SPEED_5 = 5,
    SPEED_6 = 6,
    SPEED_7 = 7,
};

#define NUM_COMPRESSOR_SPEEDS 7

#define LOWEST_OPERATING_COMPRESSOR_SPEED_INDEX 1
#define HIGHEST_OPERATING_COMPRESSOR_SPEED_INDEX 4

const float CompressorDeadTimeMeasurements[NUM_COMPRESSOR_SPEEDS] = {
    400,
    400,
    400,
    350,
    300,
    250,
    200
};

const float CompressorSpeeds[NUM_COMPRESSOR_SPEEDS] = {
    0.52,
    0.60,
    0.68,
    0.76,
    0.84,
    0.92,
    1.00
};

struct CompressorData {
    float targetTemp;
    float evaporatorInletTemp;
    float evaporatorOutletTemp;
    CompressorSpeed compressorSpeed;
};

class CompressorModel {
    public:
        void updateCompressorData(CompressorData data);
        CompressorSpeed getSpeedSetpoint();

    private:
        CompressorData data;
        CompressorData prevData;
        uint32_t compressorSpeedUpdateTime;
        float compressorPressure, prevPressure;
        float evaporatorInletTempPrev1 { 0 };
        float evaporatorInletTempPrev2 { 0 };
};
#endif