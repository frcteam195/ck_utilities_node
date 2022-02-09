#pragma once

#include <rio_control_node/Joystick.h>
#include <rio_control_node/Joystick_Status.h>
#include <thread>
#include <map>
#include <mutex>

class Joystick
{
public:
    Joystick(uint joystickID);
    static void update(const rio_control_node::Joystick_Status& joystick_status_msg);
    double getRawAxis(uint axisID);
    double getFilteredAxis(uint axisID, float deadband);
    bool getAxisActuated(uint axisID, float threshold);
    bool getButton(uint buttonID);
    bool getRisingEdgeButton(uint buttonID);
    bool getFallingEdgeButton(uint buttonID);
    int getPOV(uint povID);
private:
    static rio_control_node::Joystick_Status* joystick_status;
    bool mPrevButtonValues[16] = {0};
    int mPrevPOV = 0;
    int mJoystickID;
};


