#pragma once
#include <geometry_msgs/Transform.h>

namespace ck
{
    namespace swerve
    {
        struct WheelConfig
        {
            geometry_msgs::Transform transform; //X,Y from center of robot, angle must be current actual angle up to date
        };
    };
};