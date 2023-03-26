#include "ck_utilities/AccelRamper.hpp"
#include "ck_utilities/CKMath.hpp"
#if __has_include("ros/ros.h")
#include "ros/ros.h"

AccelRamper::AccelRamper(double accelRampRate, double decelRampRate, double zeroValue, double maxValue)
: mAccelRampRate(accelRampRate), mDecelRampRate(decelRampRate), mZeroValue(zeroValue), mMaxValue(maxValue)
{}

void AccelRamper::update_params(double accelRampRate, double decelRampRate, double zeroValue, double maxValue)
{
    mAccelRampRate = accelRampRate;
    mDecelRampRate = decelRampRate;
    mZeroValue = zeroValue;
    mMaxValue = maxValue;
}

void AccelRamper::reset()
{
    mPrevValue = mZeroValue;
}

double AccelRamper::get_value()
{
    return mPrevValue;
}

double AccelRamper::calculateOutput(double currValue)
{
    ros::Time timeNow = ros::Time::now();
    if (mPrevTime != ros::Time(0))
    {
        double dt = (timeNow - mPrevTime).toSec();
        double accelStep = dt * mAccelRampRate;
        double decelStep = dt * mDecelRampRate;
        mPrevTime = timeNow;
        // std::stringstream o;
        // o << "DT: " << dt << " accel " << accelStep << " decel " << decelStep << " prev " << mPrevValue << " curr " << currValue;
        // ROS_INFO("%s", o.str().c_str());
        if (currValue > mPrevValue && currValue >= 0)
        {
            //accelerating
            mPrevValue += accelStep;
            mPrevValue = std::min(mPrevValue, currValue);
            return mPrevValue;
        }
        else if(currValue < mPrevValue && currValue >= 0)
        {
            //decelerating
            mPrevValue -= decelStep;
            mPrevValue = std::max(mPrevValue, currValue);
            return mPrevValue;
        }
        else if (currValue < mPrevValue && currValue <= 0)
        {
            //accelerating
            mPrevValue -= accelStep;
            mPrevValue = std::max(mPrevValue, currValue);
            return mPrevValue;
        }
        else if (currValue > mPrevValue && currValue <= 0)
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

        mPrevValue = ck::math::limit(mPrevValue, mMaxValue);

    }
    mPrevTime = timeNow;
    return 0;
}
#endif