#pragma once
#ifdef ROSCPP_ROS_H
#include "ros/ros.h"
#include <string>

extern ros::NodeHandle* node;

namespace ck
{
    namespace ros
    {
        template <typename T>
        ::ros::ServiceClient& get_service_client(std::string service_name, T srv_msg)
        {
            static ::ros::ServiceClient service_client;
            if (node && !service_client)
            {
                service_client = node->serviceClient<T>(service_name, true);
            }
            return service_client;
        }
    }
}
#endif