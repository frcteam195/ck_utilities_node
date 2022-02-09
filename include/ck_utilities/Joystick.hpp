#pragma once

#include <rio_control_node/Joystick.h>
#include <rio_control_node/Joystick_Status.h>
#include <ros/ros.h>
#include <ck_utilities/Singleton.hpp>
#include <thread>
#include <map>
#include <mutex>

class JoystickHelper : public Singleton<JoystickHelper>
{
    friend Singleton;
public:

private:
    JoystickHelper();
    void joystickReceiveCallback(const rio_control_node::Joystick_Status& msg);
    rio_control_node::Joystick getJoystick(uint joystickID);
    ros::Subscriber mJoystickSubscriber;
    rio_control_node::Joystick_Status mJoystickStatus;
    std::recursive_mutex mJoystickMutex;

friend class Joystick;
};

class Joystick
{
public:
    Joystick(uint joystickID);
    double getRawAxis(uint axisID);
    double getFilteredAxis(uint axisID, float deadband);
    bool getAxisActuated(uint axisID, float threshold);
    bool getButton(uint buttonID);
    bool getRisingEdgeButton(uint buttonID);
    bool getFallingEdgeButton(uint buttonID);
    int getPOV(uint povID);
private:

    bool mPrevButtonValues[16] = {0};
    int mPrevPOV = 0;
    int mJoystickID;
};


