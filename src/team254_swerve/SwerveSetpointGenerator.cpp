#include "ck_utilities/team254_swerve/SwerveSetpointGenerator.hpp"

using ck::planners::ChassisSpeeds;
using namespace ck::team254_geometry;

namespace ck::team254_swerve
{
    SwerveSetpointGenerator::SwerveSetpointGenerator(const SwerveDriveKinematics kinematics)
        : mKinematics(kinematics)
    { }

    SwerveSetpoint SwerveSetpointGenerator::generateSetpoint(const KinematicLimits limits,
                                                             const SwerveSetpoint prevSetpoint,
                                                             planners::ChassisSpeeds desiredState,
                                                             double dt)
    {
        // TODO: Implement

        const std::vector<Translation2d> modules = mKinematics.getModuleLocations();

        std::vector<SwerveModuleState> desiredModuleStates = mKinematics.toSwerveModuleStates(desiredState);

        if (limits.kMaxDriveVelocity > 0.0)
        {
            SwerveDriveKinematics::desaturateWheelSpeeds(desiredModuleStates, limits.kMaxDriveVelocity);
            desiredState = mKinematics.toChassisSpeeds(desiredModuleStates);
        }

        bool need_to_steer = true;
        if (desiredState.toTwist2d().epsilonEquals(Twist2d::identity(), math::kEpsilon))
        {
            need_to_steer = false;
            for (size_t i = 0; i < modules.size(); i++)
            {
                desiredModuleStates.at(i).angle = prevSetpoint.mModuleStates.at(i).angle;
                desiredModuleStates.at(i).speedMetersPerSecond = 0.0;
            }
        }

        std::vector<double> prev_vx;
        std::vector<double> prev_vy;
        std::vector<Rotation2d> prev_heading;
        std::vector<double> desired_vx;
        std::vector<double> desired_vy;
        std::vector<Rotation2d> desired_heading;
        bool all_modules_should_flip = true;
        for (size_t i = 0; i < modules.size(); i++)
        {
            prev_vx.push_back(prevSetpoint.mModuleStates.at(i).angle.cos() * prevSetpoint.mModuleStates.at(i).speedMetersPerSecond);
            prev_vy.push_back(prevSetpoint.mModuleStates.at(i).angle.sin() * prevSetpoint.mModuleStates.at(i).speedMetersPerSecond);
            prev_heading.push_back(prevSetpoint.mModuleStates.at(i).angle);
            if (prevSetpoint.mModuleStates.at(i).speedMetersPerSecond < 0.0)
            {
                prev_heading.at(i) = prev_heading.at(i).flip();
            }
            desired_vx.push_back(desiredModuleStates.at(i).angle.cos() * desiredModuleStates.at(i).speedMetersPerSecond);
            desired_vy.push_back(desiredModuleStates.at(i).angle.sin() * desiredModuleStates.at(i).speedMetersPerSecond);
            desired_heading.push_back(desiredModuleStates.at(i).angle);
            if (desiredModuleStates.at(i).speedMetersPerSecond < 0.0)
            {
                desired_heading.at(i) = desired_heading.at(i).flip();
            }
            if (all_modules_should_flip)
            {
                double required_rotation_rad = std::abs(prev_heading.at(i).inverse().rotateBy(desired_heading.at(i)).getRadians());
                if (required_rotation_rad < M_PI_2)
                {
                    all_modules_should_flip = false;
                }
            }
        }

        if (all_modules_should_flip &&
            !prevSetpoint.mChassisSpeeds.toTwist2d().epsilonEquals(Twist2d::identity(), math::kEpsilon) &&
            !desiredState.toTwist2d().epsilonEquals(Twist2d::identity(), math::kEpsilon))
        {
            return generateSetpoint(limits, prevSetpoint, ChassisSpeeds(), dt);
        }

        double dx = desiredState.vxMetersPerSecond - prevSetpoint.mChassisSpeeds.vxMetersPerSecond;
        double dy = desiredState.vyMetersPerSecond - prevSetpoint.mChassisSpeeds.vyMetersPerSecond;
        double dtheta = desiredState.omegaRadiansPerSecond - prevSetpoint.mChassisSpeeds.omegaRadiansPerSecond;

        double min_s = 1.0;

        std::vector<std::optional<Rotation2d>> overrideSteering;

        const double max_theta_step = dt * limits.kMaxSteeringVelocity;
        for (size_t i = 0; i < modules.size(); i++)
        {
            if (!need_to_steer)
            {
                overrideSteering.push_back(prevSetpoint.mModuleStates.at(i).angle);
                continue;
            }
            overrideSteering.push_back(std::nullopt);
            if (math::epsilonEquals(prevSetpoint.mModuleStates.at(i).speedMetersPerSecond, 0.0))
            {
                if (math::epsilonEquals(desiredModuleStates.at(i).speedMetersPerSecond, 0.0))
                {
                    overrideSteering.at(i) = prevSetpoint.mModuleStates.at(i).angle;
                    continue;
                }

                Rotation2d necessaryRotation = prevSetpoint.mModuleStates.at(i).angle.inverse().rotateBy(
                    desiredModuleStates.at(i).angle);
                if (flipHeading(necessaryRotation))
                {
                    necessaryRotation = necessaryRotation.rotateBy(Rotation2d::kPi());
                }

                const double numStepsNeeded = std::abs(necessaryRotation.getRadians()) / max_theta_step;

                if (numStepsNeeded <= 1.0)
                {
                    overrideSteering.at(i) = desiredModuleStates.at(i).angle;
                    continue;
                }
                else
                {
                    overrideSteering.at(i) = prevSetpoint.mModuleStates.at(i).angle.rotateBy(
                        Rotation2d::fromRadians(math::signum(necessaryRotation.getRadians()) * max_theta_step));
                    min_s = 0.0;
                    continue;
                }
            }
            if (min_s == 0.0)
            {
                continue;
            }

            const int kMaxIterations = 8;
            double s = findSteeringMaxS(prev_vx.at(i), prev_vy.at(i), prev_heading.at(i).getRadians(),
                                        desired_vx.at(i), desired_vy.at(i), desired_heading.at(i).getRadians(),
                                        max_theta_step, kMaxIterations);
            min_s = std::min(min_s, s);
        }

        const double max_vel_step = dt * limits.kMaxDriveAcceleration;
        for (size_t i = 0; i < modules.size(); i++)
        {
            if (min_s == 0.0)
            {
                break;
            }
            double vx_min_s = min_s == 1.0 ? desired_vx.at(i) : (desired_vx.at(i) - prev_vx.at(i)) * min_s + prev_vx.at(i);
            double vy_min_s = min_s == 1.0 ? desired_vy.at(i) : (desired_vy.at(i) - prev_vy.at(i)) * min_s + prev_vy.at(i);
            // Find the max s for this drive wheel. Search on the interval between 0 and min_s, because we already know we can't go faster
            // than that.
            // TODO(for efficiency, do all this on v^2 to save a bunch of sqrts)
            // TODO(be smarter about root finding, since this is just a quadratic in s: ((xf-x0)*s+x0)^2+((yf-y0)*s+y0)^2)
            const int kMaxIterations = 10;
            double s = min_s * findDriveMaxS(prev_vx.at(i), prev_vy.at(i), math::hypotenuse(prev_vx.at(i), prev_vy.at(i)),
                                             vx_min_s, vy_min_s, math::hypotenuse(vx_min_s, vy_min_s),
                                             max_vel_step, kMaxIterations);
            min_s = std::min(min_s, s);
        }

        ChassisSpeeds retSpeeds(
            prevSetpoint.mChassisSpeeds.vxMetersPerSecond + min_s * dx,
            prevSetpoint.mChassisSpeeds.vyMetersPerSecond + min_s * dy,
            prevSetpoint.mChassisSpeeds.omegaRadiansPerSecond + min_s * dtheta);
        std::vector<SwerveModuleState> retStates = mKinematics.toSwerveModuleStates(retSpeeds);
        for (size_t i = 0; i < modules.size(); i++)
        {
            const std::optional<Rotation2d> maybeOverride = overrideSteering.at(i);
            if (maybeOverride.has_value())
            {
                Rotation2d ovr = maybeOverride.value();
                if (flipHeading(retStates.at(i).angle.inverse().rotateBy(ovr)))
                {
                    retStates.at(i).speedMetersPerSecond *= -1.0;
                }
                retStates.at(i).angle = ovr;
            }
            const Rotation2d deltaRotation = prevSetpoint.mModuleStates.at(i).angle.inverse().rotateBy(retStates.at(i).angle);
            if (flipHeading(deltaRotation))
            {
                retStates.at(i).angle = retStates.at(i).angle.flip();
                retStates.at(i).speedMetersPerSecond *= -1.0;
            }
        }

        return SwerveSetpoint(retSpeeds, retStates);
    }

    bool SwerveSetpointGenerator::flipHeading(Rotation2d prevToGoal) const
    {
        return std::abs(prevToGoal.getRadians()) > M_PI / 2.0;
    }

    double SwerveSetpointGenerator::unwrapAngle(double ref, double angle) const
    {
        double diff = angle - ref;
        if (diff > M_PI)
        {
            return angle - 2.0 * M_PI;
        }
        else if (diff < -M_PI)
        {
            return angle + 2.0 * M_PI;
        }
        else
        {
            return angle;
        }
    }

    double SwerveSetpointGenerator::findRoot(Function2d func,
                                             double x0,
                                             double y0,
                                             double f0,
                                             double x1,
                                             double y1,
                                             double f1,
                                             int iterations_left)
    {
        if (iterations_left < 0 || math::epsilonEquals(f0, f1))
        {
            return 1.0;
        }
        double s_guess = std::max(0.0, std::min(1.0, -f0 / (f1 - f0)));
        double x_guess = (x1 - x0) * s_guess + x0;
        double y_guess = (y1 - y0) * s_guess + y0;
        double f_guess = func.f(x_guess, y_guess);
        if (math::signum(f0) == math::signum(f_guess))
        {
            return s_guess + (1.0 - s_guess) * findRoot(func, x_guess, y_guess, f_guess, x1, y1, f1, iterations_left - 1);
        }
        else
        {
            return s_guess * findRoot(func, x0, y0, f0, x_guess, y_guess, f_guess, iterations_left - 1);
        }
    }

    double SwerveSetpointGenerator::findSteeringMaxS(double x0,
                                                     double y0,
                                                     double f0,
                                                     double x1,
                                                     double y1,
                                                     double f1,
                                                     double max_deviation,
                                                     int max_iterations)
    {
        f1 = unwrapAngle(f0, f1);
        double diff = f1 - f0;
        if (std::abs(diff) <= max_deviation)
        {
            return 1.0;
        }

        double offset = f0 + math::signum(diff) * max_deviation;
        Function2d func;
        func.f = [&](double x, double y)
        {
            return unwrapAngle(f0, std::atan2(y, x)) - offset;
        };
        return findRoot(func, x0, y0, f0 - offset, x1, y1, f1 - offset, max_iterations);
    }

    double SwerveSetpointGenerator::findDriveMaxS(double x0,
                                                  double y0,
                                                  double f0,
                                                  double x1,
                                                  double y1,
                                                  double f1,
                                                  double max_vel_step,
                                                  int max_iterations)
    {
        double diff = f1 - f0;
        if (std::abs(diff) <= max_vel_step)
        {
            return 1.0;
        }
        double offset = f0 + math::signum(diff) * max_vel_step;
        Function2d func;
        func.f = [&](double x, double y)
        {
            return math::hypotenuse(x, y) - offset;
        };
        return findRoot(func, x0, y0, f0 - offset, x1, y1, f1 - offset, max_iterations);
    }
}
