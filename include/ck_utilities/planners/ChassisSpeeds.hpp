#pragma once

#include "ck_utilities/team254_geometry/Geometry.hpp"

using namespace ck::team254_geometry;

namespace ck
{
    namespace planners
    {
        class ChassisSpeeds
        {
        public:
            ChassisSpeeds();
            ChassisSpeeds(double vxMPerSec, double vyMPerSec, double omegaRadPerSec);

            static ChassisSpeeds fromFieldRelativeSpeeds(double vxMPerSec,
                                                         double vyMPerSec,
                                                         double omegaRadPerSec,
                                                         Rotation2d robotAngle);

            static ChassisSpeeds fromRobotRelativeSpeeds(double vxMPerSec,
                                                         double vyMPerSec,
                                                         double omegaRadPerSec);

            Twist2d toTwist2d() const;

            friend std::ostream& operator<<(std::ostream& os, const ChassisSpeeds& speeds);

            double vxMetersPerSecond = 0;
            double vyMetersPerSecond = 0;
            double omegaRadiansPerSecond = 0;
        };
    }
}