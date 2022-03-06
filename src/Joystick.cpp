#include "ck_utilities/Joystick.hpp"
#include "ck_utilities/CKMath.hpp"

rio_control_node::Joystick_Status Joystick::joystick_status;
std::map<int, rio_control_node::Joystick> Joystick::joystick_map;

Joystick::Joystick(uint joystickID)
: mJoystickID(joystickID)
{
    
}

void Joystick::update(const rio_control_node::Joystick_Status& joystick_status_msg)
{
    joystick_status = joystick_status_msg;
    joystick_map.clear();
    for (const rio_control_node::Joystick& j : joystick_status_msg.joysticks)
    {
        joystick_map.emplace(j.index, j);
    }
}

double Joystick::getRawAxis(uint axisID)
{
    if (joystick_map.count(mJoystickID))
    {
        if (joystick_map[mJoystickID].axes.size() > axisID)
        {
            return joystick_status.joysticks[mJoystickID].axes[axisID];
        }
    }
    return 0;
}

double Joystick::getFilteredAxis(uint axisID, double deadband)
{
    return ck::math::normalizeWithDeadband(getRawAxis(axisID), deadband);
}

bool Joystick::getAxisActuated(uint axisID, float threshold)
{
    return getRawAxis(axisID) > threshold;
}

bool Joystick::getButton(uint buttonID)
{
    bool retVal = false;
    if (joystick_map.count(mJoystickID))
    {
        if (joystick_map[mJoystickID].buttons.size() > buttonID)
        {
            retVal = joystick_map[mJoystickID].buttons[buttonID];
        }
    }

    mPrevButtonValues[buttonID] = retVal;
    return retVal;
}

bool Joystick::getRisingEdgeButton(uint buttonID)
{
    bool currVal = false;
    if (joystick_map.count(mJoystickID))
    {
        if (joystick_map[mJoystickID].buttons.size() > buttonID)
        {
            currVal = joystick_map[mJoystickID].buttons[buttonID];
        }
    }

    bool retVal = currVal && (currVal != mPrevButtonValues[buttonID]);
    mPrevButtonValues[buttonID] = currVal;
    return retVal;
}

bool Joystick::getFallingEdgeButton(uint buttonID)
{
    bool currVal = false;
    if (joystick_map.count(mJoystickID))
    {
        if (joystick_map[mJoystickID].buttons.size() > buttonID)
        {
            currVal = joystick_map[mJoystickID].buttons[buttonID];
        }
    }

    bool retVal = !currVal && (currVal != mPrevButtonValues[buttonID]);
    mPrevButtonValues[buttonID] = currVal;
    return retVal;
}

int Joystick::getPOV(uint povID)
{
    if (joystick_map.count(mJoystickID))
    {
        if (joystick_map[mJoystickID].povs.size() > povID)
        {
            return joystick_status.joysticks[mJoystickID].povs[povID];
        }
    }
    return -1;
}