#include "ck_utilities/team254_swerve/SwerveModuleState.hpp"

using namespace ck::team254_geometry;

namespace ck
{
    namespace team254_swerve
    {
        SwerveModuleState::SwerveModuleState()
            : speedMetersPerSecond(0.0), distanceMeters(0.0), angle(Rotation2d::fromDegrees(0.0)) {}

        SwerveModuleState::SwerveModuleState(double speedMps, Rotation2d angle)
            : speedMetersPerSecond(speedMps), distanceMeters(0.0), angle(angle) {}

        SwerveModuleState::SwerveModuleState(double speedMps, double distanceM, Rotation2d angle)
            : speedMetersPerSecond(speedMps), distanceMeters(distanceM), angle(angle) {} 

    } // namespace team254_swerve
} // namespace ck
