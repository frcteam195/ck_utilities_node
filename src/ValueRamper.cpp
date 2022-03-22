#include "ck_utilities/ValueRamper.hpp"
#include "ros/ros.h"

ValueRamper::ValueRamper(double accelRampRate, double decelRampRate, double minValue, double maxValue)
: mAccelRampRate(accelRampRate), mDecelRampRate(decelRampRate), mMinValue(minValue), mMaxValue(maxValue)
{}

double ValueRamper::calculateOutput(double currValue)
{
    static ros::Time prevTime = ros::Time(0);
    ros::Time timeNow = ros::Time::now();
    if (prevTime != ros::Time(0))
    {
        double dt = (timeNow - prevTime).toSec();
        if (currValue > mPrevValue && currValue > 0)
        {
            //accelerating
            double stepSize = dt * mAccelRampRate;
            mPrevValue += stepSize;
            mPrevValue = std::min(mPrevValue, currValue);
            return mPrevValue;
        }
        else if(currValue < mPrevValue && currValue > 0)
        {
            //decelerating
            double stepSize = dt * mDecelRampRate;
            mPrevValue -= stepSize;
            mPrevValue = std::max(mPrevValue, currValue);
            return mPrevValue;
        }
        else if (currValue < mPrevValue && currValue < 0)
        {
            //accelerating
            double stepSize = dt * mAccelRampRate;
            mPrevValue -= stepSize;
            mPrevValue = std::max(mPrevValue, currValue);
            return mPrevValue;
        }
        else if (currValue > mPrevValue && currValue < 0)
        {
            //decelerating
            double stepSize = dt * mDecelRampRate;
            mPrevValue += stepSize;
            mPrevValue = std::min(mPrevValue, currValue);
            return mPrevValue;
        }
    }
    prevTime = timeNow;
    return 0;
}