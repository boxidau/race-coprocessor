#include "pwmgenerator.h"
#include "compressormodel.h"

void PWMGenerator::reset() {
    sampleCounter = 0;
    samples.clear();
    runningSum = 0;
}

uint32_t PWMGenerator::update(float value) {
    uint32_t currentSample = sampleCounter;
    sampleCounter++;
    if (sampleCounter == SAMPLE_PERIOD) {
        sampleCounter = 0;
    }

    if (currentSample == 0) {
        // average samples over the whole period, except on first run where we use the initial value
        filteredValue = samples.full() ? runningSum / SAMPLE_PERIOD : value;

        // reset min, max, target
        minIndex = getLowerBound(filteredValue);
        maxIndex = getUpperBound(filteredValue);
        float delta = CompressorSpeeds[maxIndex] - CompressorSpeeds[minIndex];
        float bottom = CompressorSpeeds[minIndex];
        float top = CompressorSpeeds[maxIndex];

        // clamp value to guarantee a duty cycle of 0%, >10%, <90%, or 100%
        float bottomFivePercent = delta * 0.05 + bottom;
        float bottomTenPercent = delta * 0.10 + bottom;
        float topTenPercent = delta * 0.90 + bottom;
        float topFivePercent = delta * 0.95 + bottom;
        if (filteredValue < bottomFivePercent) {
            filteredValue = bottom;
        } else if (filteredValue >= bottomFivePercent && value <= bottomTenPercent) {
            filteredValue = bottomTenPercent;
        } else if (filteredValue >= topTenPercent && value <= topFivePercent) {
            filteredValue = topTenPercent;
        } else if (filteredValue > topFivePercent) {
            filteredValue = top;
        }
    }

    // update filter array
    if (!samples.full()) {
        samples.push_back(value);
    } else {
        runningSum -= samples[currentSample];
        samples[currentSample] = value;
    }
    runningSum += value;

    // compare to sawtooth wave
    return filteredValue <= getSawtoothWave(currentSample) ? minIndex : maxIndex;
}

uint32_t PWMGenerator::getLowerBound(float value) {
    for (size_t i = 1; i < NUM_COMPRESSOR_SPEEDS; i++) {
        if (CompressorSpeeds[i] >= value) {
            return i - 1;
        }
    }

    return NUM_COMPRESSOR_SPEEDS - 1;
}

uint32_t PWMGenerator::getUpperBound(float value) {
    return min(getLowerBound(value) + 1, (uint32_t) (NUM_COMPRESSOR_SPEEDS - 1));
}

float PWMGenerator::getSawtoothWave(uint32_t sample) {
    return (CompressorSpeeds[maxIndex] - CompressorSpeeds[minIndex]) / SAMPLE_PERIOD * sample + CompressorSpeeds[minIndex];
}
