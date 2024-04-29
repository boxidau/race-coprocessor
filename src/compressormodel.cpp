#if 0

#include "compressormodel.h"

const float PRESSURE_TO_POWER_RATIO = 500; // W / unit
const float POWER_TO_COOLING_RATE_RATIO = (4033 * 1.5) // W / (K/s)
#define COMPRESSOR_MEASUREMENT_TEMP_LAG_TIME 10000 // ms

bool isStablePower() {
    return now - prevData.prevUpdateTime > CompressorRampTimes[data.compressorSpeed];
}

float getCompressorPressure() {
    if (data.compressorSpeed > prevData.compressorSpeed) {
        if (isStable()) {
            // we are stable
            return CompressorPressures[data.compressorSpeed];
        }

        // we went up a speed, pressure increases linearly to target
        return (CompressorPressures[data.compressorSpeed] - prevPressure) * (now - prevData.prevUpdateTime) / CompressorRampTimes[data.compressorSpeed];
    }

    // we went down a speed or switched off, exponential decay to target
    return prevPressure - (targetPressure - prevPressure) * (1 - expf(-(now - prevData.updateTime) / (2 * CompressorRampTimes[data.compressorSpeed])));
}

float getCompressorPower(float pressure) {
    return pressure * PRESSURE_TO_POWER_RATIO;
}

float getCoolingRate() {
    return (evaporatorInletTempPrev1 - evaporatorInletTempPrev2) / evaporatorInletTempTimeDelta * 1000;
}

void CompressorModel::updateCompressorData(CompressorData data) {
    if (this->data.compressorSpeed != data.compressorSpeed) {
        this->prevData = this->data;
        compressorSpeedUpdateTime = millis();
    }

    this->data = data;

    uint32_t now = millis();
    if (now >= prevTempMeasurementTime + COMPRESSOR_MEASUREMENT_TEMP_LAG_TIME) {
        evaporatorInletTempTimeDelta = now - prevTempMeasurementTime;
        prevTempMeasurementTime = now;
        evaporatorInletTempPrev2 = evaporatorInletTempPrev1;
        evaporatorInletTempPrev1 = data.evaporatorInletTemp;
    }

    // linear ramp from prior pressure to new one
    compressorPressure = getCompressorPressure();
    compressorPower = getCompressorPower(compressorPressure);

    if (isStable()) {
        // record last stable power so we can calibrate
        prevStableRate = stableRate;
        stableRate = getCoolingRate();
        compressorPowerCalibration = (stableRate - prevStableRate) / (expectedPower - prevExpectedPower);
    }
}

CompressorSpeed CompressorModel::getSpeedSetpoint() {
    float coolingRate = getCoolingRate();

    if (coolingRate >= 0) {
        // net cooling. don't change speed
        return data.compressorSpeed;
    }

    // net heating. will we hit our target temp?
    currentPower = compressorPower;
    targetPower = getCompressorPower(data.compressorSpeed);
    targetCoolingRate = coolingRate + (targetPower - currentPower) / POWER_TO_COOLING_RATE_RATIO * compressorPowerCalibration;
    if (targetCoolingRate <= 0) {
        // never. bump up a speed
        return (CompressorSpeed) min((uint32_t) data.compressorSpeed + 1, (uint32_t) CompressorSpeed::SPEED_7);
    } else {
        // when?
        timeUntilTargetTemp = x; // solve `(-mx^2/2 +cx)|(0, t) = dtemp` for t
        // do nothing
        return data.compressorSpeed;
    }
}

#endif