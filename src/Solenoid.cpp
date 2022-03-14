#include "ck_utilities/Solenoid.hpp"

#include <thread>
#include <map>
#include <mutex>
#include "ros/ros.h"
#include "rio_control_node/Solenoid_Control.h"

static std::map<uint8_t, Solenoid *> solenoid_map;
static std::recursive_mutex solenoid_mutex;

extern ros::NodeHandle * node;

Solenoid::Solenoid(uint8_t id, Solenoid::SolenoidType type)
{
    this->id = id;
    this->type = type;
}

void Solenoid::set(Solenoid::SolenoidState state)
{
    std::lock_guard<std::recursive_mutex> lock(solenoid_mutex);
    static ros::Publisher solenoid_publisher = node->advertise<rio_control_node::Solenoid_Control>("/SolenoidControl", 1);

    rio_control_node::Solenoid_Control solenoid_control;
    rio_control_node::Solenoid solenoid;
    solenoid.id = (int32_t) this->id;
    solenoid.solenoid_type = (int8_t) this->type;
    solenoid.module_type = rio_control_node::Solenoid::CTREPCM;
    solenoid.output_value = (int8_t) state;
    solenoid_control.solenoids.push_back(solenoid);

    solenoid_publisher.publish(solenoid_control);
}