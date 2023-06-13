#pragma once

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include "ck_utilities/team254_swerve/SwerveModuleState.hpp"

#include "ck_utilities/team254_geometry/Translation2d.hpp"
#include "ck_utilities/team254_geometry/Rotation2d.hpp"

#include "ck_utilities/planners/DriveMotionPlanner.hpp"

#include "Eigen/Dense"

#include <vector>

namespace ck
{
    namespace team254_swerve
    {
        class SwerveDriveKinematics
        {
        public:
            SwerveDriveKinematics(std::vector<team254_geometry::Translation2d> wheelsMeters);

            std::vector<SwerveModuleState> toSwerveModuleStates(planners::ChassisSpeeds chassisSpeeds, team254_geometry::Translation2d centerOfRotationMeters);
            std::vector<SwerveModuleState> toSwerveModuleStates(planners::ChassisSpeeds chassisSpeeds);

            planners::ChassisSpeeds toChassisSpeeds(std::vector<SwerveModuleState> wheelStates);
            planners::ChassisSpeeds toChassisSpeedWheelConstraints(std::vector<SwerveModuleState> wheelStates);

            static void desaturateWheelSpeeds(std::vector<SwerveModuleState> moduleStates, double attainableMaxSpeedMetersPerSecond);

            std::vector<team254_geometry::Translation2d> getModuleLocations() const;
            int getNumModules() const;

        private:
            Eigen::MatrixXd m_inverseKinematics;
            Eigen::MatrixXd m_forwardKinematics;

            int m_numModules;
            std::vector<team254_geometry::Translation2d> m_modules;
            team254_geometry::Translation2d m_prevCoR;
            std::vector<team254_geometry::Rotation2d> m_rotations;
        };
    } // namespace team254_swerve
} // namespace ck
