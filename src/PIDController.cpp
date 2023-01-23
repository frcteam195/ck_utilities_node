#include "ck_utilities/PIDController.hpp"

namespace ck
{
    PIDController::PIDController()
    {
        setGains(0.0, 0.0, 0.0);
        error = 0.0;
        errorSum = 0.0;
        lastError = 0.0;
        errorD = 0.0;
        lastTimestamp = 0.0;
    } 

    PIDController::PIDController(double kP, double kI, double kD)
    {
        setGains(kP, kI, kD);
        error = 0.0;
        errorSum = 0.0;
        lastError = 0.0;
        lastTimestamp = 0.0;
    }

    PIDController::~PIDController()
    {
        if (pidTuner != nullptr)
        {
            delete pidTuner;
        }
    }

    void PIDController::setGains(double kP, double kI, double kD)
    {
        this->kP = kP;
        this->kI = kI;
        this->kD = kD;
    }

    double PIDController::update(double setpoint, double actual, double timestamp)
    {
        return update(setpoint - actual, timestamp);
    }

    double PIDController::update(double error, double timestamp)
    {
        if (pidTuner != nullptr)
        {
            pidTuner->update();
        }

        double dt = lastTimestamp > 0 ? timestamp - lastTimestamp : 1.0;
        lastTimestamp = timestamp;

        errorSum += error * dt;
        errorD = (error - lastError) / dt;
        lastError = error;

        return error * kP + errorSum * kI + errorD * kD;
    }

    void PIDController::initTuner(ros::NodeHandle *n, std::string topic_basename)
    {
        pidTuner = new PIDTuner(n, topic_basename, this);
    }
} // namespace ck
