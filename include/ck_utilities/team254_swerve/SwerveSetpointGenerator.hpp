#pragma once

#include "ck_utilities/team254_geometry/Geometry.hpp"
#include "ck_utilities/team254_swerve/SwerveModuleState.hpp"
#include "ck_utilities/team254_swerve/SwerveSetpoint.hpp"
#include "ck_utilities/team254_swerve/SwerveDriveKinematics.hpp"

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

        class Function2d
        {
        public:
            std::function<double(double, double)> f;
        };

        class SwerveSetpointGenerator
        {
        public:
            SwerveSetpointGenerator(const SwerveDriveKinematics kinematics);

            SwerveSetpoint generateSetpoint(const KinematicLimits limits, const SwerveSetpoint prevSetpoint, planners::ChassisSpeeds desiredState, double dt);

        private:
            SwerveDriveKinematics mKinematics;

            bool flipHeading(team254_geometry::Rotation2d prevToGoal) const;
            double unwrapAngle(double ref, double angle) const;

            double findRoot(Function2d func, double x0, double y0, double f0, double x1, double y1, double f1, int iterations_left);

            double findSteeringMaxS(double x0, double y0, double f0, double x1, double y1, double f1, double max_deviation, int max_iterations);
            double findDriveMaxS(double x0, double y0, double f0, double x1, double y1, double f1, double max_vel_step, int max_iterations);
        };
    } // namespace team254_swerve
} // namespace ck
