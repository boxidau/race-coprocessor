#include <Arduino.h>
#include <Array.h>

#define SAMPLE_PERIOD 100 // 10s

class PWMGenerator {
    public:
        void reset();
        uint32_t update(float value);

    private:
        uint32_t getLowerBound(float value);
        uint32_t getUpperBound(float value);
        float getSawtoothWave(uint32_t sample);

        uint32_t sampleCounter { 0 };
        uint32_t minIndex { 0 };
        uint32_t maxIndex { 0 };
        float filteredValue { 0 };
        float runningSum { 0 };
        Array<float, SAMPLE_PERIOD> samples;
};
