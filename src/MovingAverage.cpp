#include "ck_utilities/MovingAverage.hpp"
#include <cstring>
#include <iostream>

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
        mLastBufferValue = mData[mCurrentIndex];

        if (++mCurrentIndex >= mSizeOfBuffer)
        {
            mCurrentIndex = 0;
        }

        mOverwrittenBufferValue = mData[mCurrentIndex];
        mPlacingValue = sample + mLastBufferValue;
        if(mCurrentIndex == 0)
        {
            mAveragingBufferOffset = mOverwrittenBufferValue;
            mPlacingValue -= mAveragingBufferOffset;
        }
        mNumOfValidSamples = ++mNumOfValidSamples > mSizeOfBuffer ? mSizeOfBuffer : mNumOfValidSamples;

        mData[mCurrentIndex] = mPlacingValue;
        mCurrentAverage = (mPlacingValue - mOverwrittenBufferValue + mAveragingBufferOffset) / (float)mNumOfValidSamples;
        return mCurrentAverage;
    }

    double MovingAverage::getAverage()
    {
        return mCurrentAverage;
    }
}