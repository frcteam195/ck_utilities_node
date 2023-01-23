#pragma once

#include "ck_utilities/PIDController.hpp"

#include "ck_ros_base_msgs_node/PID_Tuning.h"
#include "ros/ros.h"

#include <string>

namespace ck
{
    class PIDController;

    class PIDTuner
    {
    public:
        PIDTuner(ros::NodeHandle *n, std::string topic_basename, PIDController *pid);

        void update();

    private:
        void set_gains_callback(const ck_ros_base_msgs_node::PID_Tuning &tuning);

        ros::NodeHandle *n;
        std::string topic_basename;
        PIDController *pid;

        ros::Publisher actual_gains_pub;
        ros::Publisher set_gains_pub;
        ros::Subscriber set_gains_sub;
    };
} // namespace ck
