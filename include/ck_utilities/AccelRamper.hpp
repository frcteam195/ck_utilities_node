#pragma once
#if __has_include("ros/ros.h")
#include "ros/ros.h"

/**
 * @brief A class to ramp a value from one place to another.
 * Ramp rate is given in seconds from full off to full on and full on to full off
 * The minimum value is not the most negative value, but rather the value at 0
 */
class AccelRamper
{
public:
    AccelRamper(double accelRampRate, double decelRampRate, double zeroValue, double maxValue);
    double calculateOutput(double currValue);
    double get_value();
    void update_params(double accelRampRate, double decelRampRate, double zeroValue, double maxValue);
    void reset();

private:
    double mAccelRampRate = 0;
    double mDecelRampRate = 0;
    double mZeroValue = 0;
    double mMaxValue = 0;
    ros::Time mPrevTime = ros::Time(0);

    double mPrevValue = 0;
};
#endif