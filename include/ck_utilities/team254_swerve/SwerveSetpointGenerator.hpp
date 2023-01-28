#pragma once

#include "ck_utilities/team254_geometry/Geometry.hpp"
#include "ck_utilities/team254_swerve/SwerveModuleState.hpp"
#include "ck_utilities/team254_swerve/SwerveSetpoint.hpp"

#include <vector>

namespace ck
{
    namespace team254_swerve
    {
        struct KinematicLimits
        {
            double kMaxDriveVelocity; // m/s
            double kMaxDriveAcceleration; // m/s^2
            double kMaxSteeringVelocity; // rad/s
        };

        class SwerveSetpointGenerator
        {
            
        };
    } // namespace team254_swerve
} // namespace ck
