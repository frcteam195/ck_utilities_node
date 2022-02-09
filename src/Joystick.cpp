#include "ck_utilities/Joystick.hpp"
#include "ck_utilities/CKMath.hpp"

extern ros::NodeHandle* node;

JoystickHelper::JoystickHelper()
{
    std::lock_guard<std::recursive_mutex> lock(mJoystickMutex);
    mJoystickSubscriber = node->subscribe("JoystickStatus", 10, &JoystickHelper::joystickReceiveCallback, this);
    
}

void JoystickHelper::joystickReceiveCallback(const rio_control_node::Joystick_Status& msg)
{
    std::lock_guard<std::recursive_mutex> lock(mJoystickMutex);
    mJoystickStatus = msg;
}

rio_control_node::Joystick JoystickHelper::getJoystick(uint joystickID)
{
    if (mJoystickStatus.joysticks.size() > (joystickID + 1))
    {
        return mJoystickStatus.joysticks[joystickID];
    }
    ROS_WARN("Joystick ID: %d not found!", joystickID);
    return rio_control_node::Joystick();
}


Joystick::Joystick(uint joystickID)
: mJoystickID(joystickID)
{
    JoystickHelper::getInstance();
}

double Joystick::getRawAxis(uint axisID)
{
    return JoystickHelper::getInstance().getJoystick(mJoystickID).axes[axisID];
}

double Joystick::getFilteredAxis(uint axisID, float deadband)
{
    double val = getRawAxis(axisID);
    val = (std::fabs(val) > std::fabs(deadband)) ? val : 0.0;

    if (val != 0)
    {
        val = ck::math::signum(val) * ((std::fabs(val) - deadband) / (1.0 - deadband));
    }

    return (std::fabs(val) > std::fabs(deadband)) ? val : 0.0;
}

bool Joystick::getAxisActuated(uint axisID, float threshold)
{
    return JoystickHelper::getInstance().getJoystick(mJoystickID).axes[axisID] > threshold;
}

bool Joystick::getButton(uint buttonID)
{
    bool retVal = JoystickHelper::getInstance().getJoystick(mJoystickID).buttons[buttonID];
    mPrevButtonValues[buttonID] = retVal;
    return retVal;
}

bool Joystick::getRisingEdgeButton(uint buttonID)
{
    bool currVal = JoystickHelper::getInstance().getJoystick(mJoystickID).buttons[buttonID];
    bool retVal = currVal && (currVal != mPrevButtonValues[buttonID]);
    mPrevButtonValues[buttonID] = retVal;
    return retVal;
}

bool Joystick::getFallingEdgeButton(uint buttonID)
{
    bool currVal = JoystickHelper::getInstance().getJoystick(mJoystickID).buttons[buttonID];
    bool retVal = !currVal && (currVal != mPrevButtonValues[buttonID]);
    mPrevButtonValues[buttonID] = retVal;
    return retVal;
}

int Joystick::getPOV(uint povID)
{
    return JoystickHelper::getInstance().getJoystick(mJoystickID).povs[povID];
}