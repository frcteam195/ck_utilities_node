#pragma once

#include "ck_utilities/PIDTuner.hpp"

#include "ros/ros.h"

#include <string>
#include <atomic>

namespace ck
{
    class PIDTuner;

    class PIDController
    {
    public:
        friend class PIDTuner;

        PIDController(double kP=0.0, double kI=0.0, double kD=0.0, double filter_r=0.0);

        ~PIDController();
        
        void setGains(double kP, double kI, double kD);
        void setGains(double kP, double kI, double kD, double filter_r);

        double update(double setpoint, double actual);
        double update(double error);

        void initTuner(ros::NodeHandle *n, std::string topic_basename);

    private:
        std::atomic<double> kP{0.0};
        std::atomic<double> kI{0.0};
        std::atomic<double> kD{0.0};
        std::atomic<double> filter_r{0.0};
        double error;
        double errorSum;
        double lastError;
        double errorD;

        PIDTuner *pidTuner = nullptr;
    };
} // namespace ck
