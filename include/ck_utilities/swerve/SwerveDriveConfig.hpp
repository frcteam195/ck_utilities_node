#pragma once

#include <vector>
#include "WheelConfig.hpp"
namespace ck
{
    namespace swerve
    {
        struct SwerveDriveConfig
        {
        public:
            std::vector<WheelConfig> wheels;
        };
    };
};