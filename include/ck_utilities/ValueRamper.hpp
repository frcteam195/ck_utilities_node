#pragma once

/**
 * @brief A class to ramp a value from one place to another.
 * Ramp rate is given in seconds from full off to full on and full on to full off
 * 
 */
class ValueRamper
{
public:
    ValueRamper(double accelRampRate, double decelRampRate, double minValue, double maxValue);
    double calculateOutput(double currValue);

private:
    double mAccelRampRate = 0;
    double mDecelRampRate = 0;
    double mMinValue = 0;
    double mMaxValue = 0;
    
    double mPrevValue = 0;
};