#pragma once

#include "ck_utilities/team254_geometry/Rotation2d.hpp"
#include "ck_utilities/CKMath.hpp"

namespace ck
{
    namespace team254_swerve
    {
        class SwerveModuleState
        {
        public:
            SwerveModuleState();
            SwerveModuleState(double speedMps, team254_geometry::Rotation2d angle);
            SwerveModuleState(double speedMps, double distanceM, team254_geometry::Rotation2d angle);

            friend std::ostream& operator<<(std::ostream& os, const SwerveModuleState& state);

            double speedMetersPerSecond;
            double distanceMeters;
            team254_geometry::Rotation2d angle;
        };
    } // namespace team254_swerve
} // namespace ck
