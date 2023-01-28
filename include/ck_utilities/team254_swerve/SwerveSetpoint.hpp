#pragma once

#include "ck_utilities/planners/DriveMotionPlanner.hpp"
#include "ck_utilities/team254_swerve/SwerveModuleState.hpp"

#include <vector>

namespace ck
{
    namespace team254_swerve
    {
        class SwerveSetpoint
        {
        public:
            SwerveSetpoint(planners::ChassisSpeeds chassisSpeeds, std::vector<SwerveModuleState> initialStates)
            {
                mChassisSpeeds = chassisSpeeds;
                mModuleStates = initialStates;
            }

            planners::ChassisSpeeds mChassisSpeeds;
            std::vector<SwerveModuleState> mModuleStates;
        };
    } // namespace team254_swerve
} // namespace ck
