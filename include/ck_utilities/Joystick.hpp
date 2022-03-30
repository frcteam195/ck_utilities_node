#pragma once

#include <rio_control_node/Joystick.h>
#include <rio_control_node/Joystick_Status.h>
#include <thread>
#include <map>
#include <mutex>

#define MAX_NUM_BUTTONS 16

class Joystick
{
public:
    Joystick(uint joystickID);
    static void update(const rio_control_node::Joystick_Status& joystick_status_msg);
    double getRawAxis(uint axisID);
    double getFilteredAxis(uint axisID, double deadband);
    bool getAxisActuated(uint axisID, float threshold);
    bool getButton(uint buttonID);
    bool getRisingEdgeButton(uint buttonID);
    bool getFallingEdgeButton(uint buttonID);
    int getPOV(uint povID);
private:
    static rio_control_node::Joystick_Status joystick_status;
    static std::map<int, rio_control_node::Joystick>  joystick_map;
    bool mPrevButtonValues[MAX_NUM_BUTTONS] = {0};
    int mPrevPOV = 0;
    int mJoystickID;
};


