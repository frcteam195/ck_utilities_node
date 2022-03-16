#pragma once

#include <cstdint>
#include <mutex>
#include <atomic>

namespace ck
{
    class MovingAverage
    {
    public:
        MovingAverage(uint32_t maxNumSamples);
        ~MovingAverage();
        double addSample(double sample);
        double getAverage();

    private:
        uint32_t mCurrentIndex = 0;
        uint32_t mSizeOfBuffer = 0;
        uint32_t mNumOfValidSamples = 0;
        std::atomic<double> mCurrentAverage = 0;
        double* mData = nullptr;
        double mLastVelocityBufferValue = 0;
        double mOverwrittenVelocityBufferValue = 0;
        double mAveragingBufferOffset = 0;
        double mPlacingValue = 0;

        std::recursive_mutex mBufferLock;
    };
};