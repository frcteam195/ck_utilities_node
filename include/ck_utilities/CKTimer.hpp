#pragma once

#include <chrono>
#include <mutex>
#include <math.h>

namespace ck
{
    class ElapsedTimer {
    public:
        ElapsedTimer();
        void start();
        double hasElapsed();
    private:
#ifdef __APPLE__
        std::chrono::high_resolution_clock::time_point startTime;
#else
        std::chrono::_V2::high_resolution_clock::time_point startTime;
#endif
    };

    class TimeoutTimer {
    public:
        TimeoutTimer(double timeout);
        bool isTimedOut();
        void reset();
        double getTimeLeft();
        double getTimeoutPeriod() const;

    private:
        double timeout;
        bool firstRun;
        ElapsedTimer eTimer;

        std::mutex mtx;

        void setFirstRun(bool firstRun);
    };
};