#pragma once

#include <vector>
#include "Wheel.hpp"
namespace ck
{
    namespace swerve
    {
        struct SwerveDriveOutput
        {
        public:
            std::vector<Wheel> wheels;
        };
    };
};