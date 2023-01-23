#pragma once

#include "ck_utilities/PIDTuner.hpp"

#include "ros/ros.h"

#include <string>

namespace ck
{
    class PIDTuner;

    class PIDController
    {
    public:
        friend class PIDTuner;

        PIDController();
        PIDController(double kP, double kI, double kD);

        ~PIDController();
        
        void setGains(double kP, double kI, double kD);

        double update(double setpoint, double actual, double timeStamp);
        double update(double error, double timeStamp);

        void initTuner(ros::NodeHandle *n, std::string topic_basename);

    private:
        double kP;
        double kI;
        double kD;
        double error;
        double errorSum;
        double lastError;
        double errorD;
        double lastTimestamp;

        PIDTuner *pidTuner = nullptr;
    };
} // namespace ck
