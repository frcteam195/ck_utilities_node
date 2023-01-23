#include "ck_utilities/PIDTuner.hpp"

namespace ck
{
    PIDTuner::PIDTuner(ros::NodeHandle *n, std::string topic_basename, PIDController *pid)
    {
        this->n = n;
        this->topic_basename = topic_basename;
        this->pid = pid;

        actual_gains_pub = n->advertise<ck_ros_base_msgs_node::PID_Tuning>(
            "/" + topic_basename + "GainsActual",
            10);

        std::string set_gains_topic = "/" + topic_basename + "GainsSet";

        set_gains_pub = n->advertise<ck_ros_base_msgs_node::PID_Tuning>(
            set_gains_topic,
            10);

        set_gains_sub = n->subscribe(
            set_gains_topic,
            10,
            &PIDTuner::set_gains_callback,
            this,
            ros::TransportHints().tcpNoDelay());

        ROS_ERROR("TOPIC NAME: %s", set_gains_topic.c_str());
    }

    void PIDTuner::update()
    {
        ck_ros_base_msgs_node::PID_Tuning tuning;
        tuning.kP = pid->kP;
        tuning.kI = pid->kI;
        tuning.kD = pid->kD;
        tuning.filter_r = 0.0;

        actual_gains_pub.publish(tuning);
    }

    void PIDTuner::set_gains_callback(const ck_ros_base_msgs_node::PID_Tuning &tuning)
    {
        ROS_ERROR("Got gains %0.2f, %0.2f, %0.2f", tuning.kP, tuning.kI, tuning.kD);

        pid->setGains(tuning.kP, tuning.kI, tuning.kD);
    }
} // namespace ck
