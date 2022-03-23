#include "ck_utilities/ValueRamper.hpp"
#include "ros/ros.h"

ValueRamper::ValueRamper(double accelRampRate, double decelRampRate, double zeroValue, double maxValue)
: mAccelRampRate(accelRampRate), mDecelRampRate(decelRampRate), mZeroValue(zeroValue), mMaxValue(maxValue)
{}

double ValueRamper::calculateOutput(double currValue)
{
    static ros::Time prevTime = ros::Time(0);
    ros::Time timeNow = ros::Time::now();
    if (prevTime != ros::Time(0))
    {
        double dt = (timeNow - prevTime).toSec();
        double accelStep = (mMaxValue - mZeroValue) * dt / mAccelRampRate;
        double decelStep = (mMaxValue - mZeroValue) * dt / mDecelRampRate;
        if (currValue > mPrevValue && currValue > 0)
        {
            //accelerating
            mPrevValue += accelStep;
            mPrevValue = std::min(mPrevValue, currValue);
            return mPrevValue;
        }
        else if(currValue < mPrevValue && currValue > 0)
        {
            //decelerating
            mPrevValue -= decelStep;
            mPrevValue = std::max(mPrevValue, currValue);
            return mPrevValue;
        }
        else if (currValue < mPrevValue && currValue < 0)
        {
            //accelerating
            mPrevValue -= accelStep;
            mPrevValue = std::max(mPrevValue, currValue);
            return mPrevValue;
        }
        else if (currValue > mPrevValue && currValue < 0)
        {
            //decelerating
            mPrevValue += decelStep;
            mPrevValue = std::min(mPrevValue, currValue);
            return mPrevValue;
        }
        else
        {
            mPrevValue = currValue;
            return currValue;
        }
    }
    prevTime = timeNow;
    return 0;
}