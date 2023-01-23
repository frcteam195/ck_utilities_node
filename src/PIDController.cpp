#include "ck_utilities/PIDController.hpp"

namespace ck
{
    PIDController::PIDController(double kP, double kI, double kD, double filter_r)
    {
        setGains(kP, kI, kD);
        this->filter_r = filter_r;
        error = 0.0;
        errorSum = 0.0;
        lastError = 0.0;
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

    void PIDController::setGains(double kP, double kI, double kD, double filter_r)
    {
        // TODO: suck it <3
        setGains(kP, kI, kD);
        this->filter_r = filter_r;
    }

    double PIDController::update(double setpoint, double actual)
    {
        return update(setpoint - actual);
    }

    double PIDController::update(double error)
    {
        errorSum += error;
        errorD += (1 - filter_r) * (error - lastError);
        lastError = error;

        return error * kP + errorSum * kI + errorD * kD;
    }

    void PIDController::initTuner(ros::NodeHandle *n, std::string topic_basename)
    {
        pidTuner = new PIDTuner(n, topic_basename, this);
    }
} // namespace ck
