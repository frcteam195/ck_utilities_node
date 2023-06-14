#include "ck_utilities/planners/ChassisSpeeds.hpp"

namespace ck
{
    namespace planners
    {

        ChassisSpeeds::ChassisSpeeds() {}

        ChassisSpeeds::ChassisSpeeds(double vxMPerSec, double vyMPerSec, double omegaRadPerSec)
        {
            vxMetersPerSecond = vxMPerSec;
            vyMetersPerSecond = vyMPerSec;
            omegaRadiansPerSecond = omegaRadPerSec;
        }

        ChassisSpeeds ChassisSpeeds::fromFieldRelativeSpeeds(double vxMPerSec,
                                                                double vyMPerSec,
                                                                double omegaRadPerSec,
                                                                Rotation2d robotAngle)
        {
            return ChassisSpeeds(
                vxMPerSec * robotAngle.cos() + vyMPerSec * robotAngle.sin(),
                -vxMPerSec * robotAngle.sin() + vyMPerSec * robotAngle.cos(),
                omegaRadPerSec);
        }

        ChassisSpeeds ChassisSpeeds::fromRobotRelativeSpeeds(double vxMPerSec,
                                                        double vyMPerSec,
                                                        double omegaRadPerSec)
        {
            return ChassisSpeeds(vxMPerSec, vyMPerSec, omegaRadPerSec);
        }

        Twist2d ChassisSpeeds::toTwist2d() const
        {
            return Twist2d(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        }

        std::ostream& operator<<(std::ostream& os, const ChassisSpeeds& speeds)
        {
            os << "ChassisSpeeds(";
            os << "Vx: " << speeds.vxMetersPerSecond << " m/s, ";
            os << "Vy: " << speeds.vyMetersPerSecond << " m/s, ";
            os << "Omega: " << speeds.omegaRadiansPerSecond << " rad/s)";
            return os;
        }

    }

}