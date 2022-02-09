#include "ck_utilities/Joystick.hpp"
#include "ck_utilities/CKMath.hpp"

rio_control_node::Joystick_Status* Joystick::joystick_status;

Joystick::Joystick(uint joystickID)
: mJoystickID(joystickID)
{
    
}

void Joystick::update(const rio_control_node::Joystick_Status& joystick_status_msg)
{
    joystick_status = &((rio_control_node::Joystick_Status&)joystick_status_msg);
}

double Joystick::getRawAxis(uint axisID)
{
    return joystick_status->joysticks[mJoystickID].axes[axisID];
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
    return joystick_status->joysticks[mJoystickID].axes[axisID] > threshold;
}

bool Joystick::getButton(uint buttonID)
{
    bool retVal = joystick_status->joysticks[mJoystickID].buttons[buttonID];
    mPrevButtonValues[buttonID] = retVal;
    return retVal;
}

bool Joystick::getRisingEdgeButton(uint buttonID)
{
    bool currVal = joystick_status->joysticks[mJoystickID].buttons[buttonID];
    bool retVal = currVal && (currVal != mPrevButtonValues[buttonID]);
    mPrevButtonValues[buttonID] = currVal;
    return retVal;
}

bool Joystick::getFallingEdgeButton(uint buttonID)
{
    bool currVal = joystick_status->joysticks[mJoystickID].buttons[buttonID];
    bool retVal = !currVal && (currVal != mPrevButtonValues[buttonID]);
    mPrevButtonValues[buttonID] = currVal;
    return retVal;
}

int Joystick::getPOV(uint povID)
{
    return joystick_status->joysticks[mJoystickID].povs[povID];
}