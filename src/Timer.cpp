#include "Timer.hpp"

namespace ck{
    ElapsedTimer::ElapsedTimer() {}
    void ElapsedTimer::start() {
        startTime = std::chrono::high_resolution_clock::now();
    }

    double ElapsedTimer::hasElapsed() {
        std::chrono::duration<double, std::nano> d = (std::chrono::high_resolution_clock::now() - startTime);
        return d.count() / 1000000000.0;
    }

    TimeoutTimer::TimeoutTimer(double timeout)
    {
        this->timeout = timeout;
        setFirstRun(true);
    }

    bool TimeoutTimer::isTimedOut()
    {
        if (firstRun) {
            eTimer.start();
            setFirstRun(false);
        }
        return eTimer.hasElapsed() > timeout;
    }

    void TimeoutTimer::reset()
    {
        setFirstRun(true);
    }

    double TimeoutTimer::getTimeLeft()
    {
        return std::fmax(timeout - eTimer.hasElapsed(), 0.0);
    }

    double TimeoutTimer::getTimeoutPeriod() const
    {
        return timeout;
    }

    void TimeoutTimer::setFirstRun(bool firstRun)
    {
        std::lock_guard<std::mutex> lock(mtx);
        this->firstRun = firstRun;
    }
};