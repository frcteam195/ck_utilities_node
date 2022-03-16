#include "ck_utilities/MovingAverage.hpp"
#include <cstring>

namespace ck
{

    MovingAverage::MovingAverage(uint32_t maxNumSamples)
    {
        mSizeOfBuffer = maxNumSamples;
        mData = (double*)malloc(maxNumSamples * sizeof(double));
        memset(mData, 0, maxNumSamples * sizeof(double));
    }

    MovingAverage::~MovingAverage()
    {
        free(mData);
    }

    double MovingAverage::addSample(double sample)
    {
        std::lock_guard<std::recursive_mutex> lock(mBufferLock);
        mLastVelocityBufferValue = mData[mCurrentIndex];
        mOverwrittenVelocityBufferValue = mData[++mCurrentIndex];
        mPlacingValue = sample + mLastVelocityBufferValue;
        if(mCurrentIndex == 0)
        {
            mAveragingBufferOffset = mOverwrittenVelocityBufferValue;
            mPlacingValue -= mAveragingBufferOffset;
        }
        mNumOfValidSamples = ++mNumOfValidSamples > mSizeOfBuffer ? mSizeOfBuffer : mNumOfValidSamples;

        mData[mCurrentIndex] = mPlacingValue;
        mCurrentAverage = (mPlacingValue - mOverwrittenVelocityBufferValue + mAveragingBufferOffset) / (float)mNumOfValidSamples;
        return mCurrentAverage;
    }

    double MovingAverage::getAverage()
    {
        return mCurrentAverage;
    }
}