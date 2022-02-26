#include "ck_utilities/Piston.hpp"

#include <thread>
#include <map>
#include <mutex>
#include "ros/ros.h"

#include "rio_control_node/Solenoid_Control.h"

static std::map<uint8_t, Piston *> piston_map;
static std::recursive_mutex piston_mutex;

extern ros::NodeHandle * node;

Piston::Piston(uint8_t id, Piston::PistonType type)
{
    this->id = id;
    this->type = type;
}

void Piston::set(Piston::PistonState state)
{
    std::lock_guard<std::recursive_mutex> lock(piston_mutex);
    static ros::Publisher piston_publisher = node->advertise<rio_control_node::Solenoid_Control>("/SolenoidControl", 1);

    rio_control_node::Solenoid_Control solenoid_control;
    rio_control_node::Solenoid solenoid;
    solenoid.id = (int32_t) this->id;
    solenoid.solenoid_type = (int8_t) this->type;
    solenoid.module_type = rio_control_node::Solenoid::REVPH;
    solenoid.output_value = (int8_t) state;
    solenoid_control.solenoids.push_back(solenoid);

    piston_publisher.publish(solenoid_control);
}