#pragma once

/**
 * @brief A class to ramp a value from one place to another.
 * Ramp rate is given in seconds from full off to full on and full on to full off
 * The minimum value is not the most negative value, but rather the value at 0
 */
class ValueRamper
{
public:
    ValueRamper(double accelRampRate, double decelRampRate, double zeroValue, double maxValue);
    double calculateOutput(double currValue);

private:
    double mAccelRampRate = 0;
    double mDecelRampRate = 0;
    double mZeroValue = 0;
    double mMaxValue = 0;

    double mPrevValue = 0;
};