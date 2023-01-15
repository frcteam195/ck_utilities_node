#include "ck_utilities/planners/DriveMotionPlanner.hpp"

#include "ck_utilities/physics/DCMotorTransmission.hpp"
#include "ck_utilities/trajectory/TrajectoryUtil.hpp"
#include "ck_utilities/trajectory/timing/CentripetalAccelerationConstraint.hpp"
#include "ck_utilities/trajectory/timing/DifferentialDriveDynamicsConstraint.hpp"
#include "ck_utilities/trajectory/timing/TimingUtil.hpp"
#include "ck_utilities/trajectory/timing/TimingConstraint.hpp"

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

        Output::Output(double leftVelocity,
                   double rightVelocity,
                   double leftAcceleration,
                   double rightAcceleration,
                   double leftFeedForwardVoltage,
                   double rightFeedForwardVoltage)
        {
            mLeftVelocity = leftVelocity;
            mRightVelocity = rightVelocity;

            mLeftAcceleration = leftAcceleration;
            mRightAcceleration = rightAcceleration;

            mLeftFeedForwardVoltage = leftFeedForwardVoltage;
            mRightFeedForwardVoltage = rightFeedForwardVoltage;
        }

        void Output::flip(void)
        {
            double tempLeftVelocity = mLeftVelocity;
            mLeftVelocity = -mRightVelocity;
            mRightVelocity = -tempLeftVelocity;

            double tempLeftAcceleration = mLeftAcceleration;
            mLeftAcceleration = -mRightAcceleration;
            mRightAcceleration = -tempLeftAcceleration;

            double tempLeftFeedForwardVoltage = mLeftFeedForwardVoltage;
            mLeftFeedForwardVoltage = -mRightFeedForwardVoltage;
            mRightFeedForwardVoltage = -tempLeftFeedForwardVoltage;
        }

        void Output::setZeros(void)
        {
            mLeftVelocity = 0.0;
            mRightVelocity = 0.0;

            mLeftAcceleration = 0.0;
            mRightAcceleration = 0.0;

            mLeftFeedForwardVoltage = 0.0;
            mRightFeedForwardVoltage = 0.0;
        }

        DriveMotionPlanner::DriveMotionPlanner(void)
            : mSpeedLookahead(kAdaptivePathPlannerMinLookaheadDistance, kAdaptivePathPlannerMaxLookaheadDistance, 0.0, math::meters_to_inches(kMaxVelocityMetersPerSecond))
        {
            mDt = 0.0;

            double wheelRadiusMeters = math::inches_to_meters(K_DRIVE_WHEEL_RADIUS_INCHES);
            double wheelRadiusSquaredMeters = wheelRadiusMeters * wheelRadiusMeters;
            double torquePerVolt = wheelRadiusSquaredMeters * K_ROBOT_LINEAR_INERTIA / (2.0 * K_DRIVE_KA);

            physics::DCMotorTransmission transmission = physics::DCMotorTransmission(1.0 / K_DRIVE_KV, torquePerVolt, K_DRIVE_V_INTERCEPT);
            (void)transmission;

            double effectiveWheelbaseRadius = math::inches_to_meters(K_DRIVE_WHEEL_TRACK_WIDTH_INCHES / 2.0 * K_TRACK_SCRUB_FACTOR);
            (void)effectiveWheelbaseRadius;
        }

        Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> DriveMotionPlanner::generateTrajectory(bool reversed,
                                                                                                                   std::vector<Pose2d> waypoints,
                                                                                                                   std::vector<Rotation2d> headings,
                                                                                                                   double maximumVelocity,     // Inches per Second
                                                                                                                   double maximumAcceleration, // Inches per Second^2
                                                                                                                   double maximumVoltage)
        {
            return this->generateTrajectory(reversed,
                                            waypoints,
                                            headings,
                                            0.0,
                                            0.0,
                                            maximumVelocity,
                                            maximumAcceleration,
                                            maximumVoltage);
        }

        Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> DriveMotionPlanner::generateTrajectory(bool reversed,
                                                                                                                   std::vector<Pose2d> waypoints,
                                                                                                                   std::vector<Rotation2d> headings,
                                                                                                                   double startVelocity,       // Inches per Second
                                                                                                                   double endVelocity,         // Inches per Second
                                                                                                                   double maximumVelocity,     // Inches per Second
                                                                                                                   double maximumAcceleration, // Inches per Second^2
                                                                                                                   double maximumVoltage)
        {
            (void)maximumVoltage;
            std::vector<Pose2d> waypoints_maybe_flipped(waypoints);
            std::vector<Rotation2d> headings_maybe_flipped(headings);
            const Pose2d flip = Pose2d::fromRotation(Rotation2d(-1, 0, false));

            if (reversed)
            {
                waypoints_maybe_flipped.clear();
                headings_maybe_flipped.clear();
                for (size_t i = 0; i < waypoints.size(); i++)
                {
                    waypoints_maybe_flipped.push_back(waypoints[i].transformBy(flip));
                    headings_maybe_flipped.push_back(headings[i].rotateBy(flip.getRotation()));
                }
            }

            Trajectory<Pose2dWithCurvature, Rotation2d> trajectory = TrajectoryUtil::trajectoryFromWaypoints(
                waypoints_maybe_flipped, headings_maybe_flipped, kMaxDx, kMaxDy, kMaxDTheta);
            
            if (reversed)
            {
                std::vector<Pose2dWithCurvature> flipped_points(trajectory.length());
                std::vector<Rotation2d> flipped_headings(trajectory.length());
                for (int i = 0; i < trajectory.length(); i++)
                {
                    flipped_points.push_back(Pose2dWithCurvature(trajectory.getState(i).getPose().transformBy(flip),
                                                                 -trajectory.getState(i).getCurvature(),
                                                                 trajectory.getState(i).getDCurvatureDs()));
                    flipped_headings.push_back(Rotation2d(trajectory.getHeading(i).rotateBy(flip.getRotation())));
                }
                trajectory = Trajectory<Pose2dWithCurvature, Rotation2d>(flipped_points, flipped_headings);
            }

            std::vector<TimingConstraint<Pose2dWithCurvature> *> constraints;

            DistanceView<Pose2dWithCurvature, Rotation2d> distance_view(trajectory);
            Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> timed_trajectory = TimingUtil::timeParameterizeTrajectory(reversed,
                                                                                                                                          distance_view,
                                                                                                                                          kMaxDx,
                                                                                                                                          constraints,
                                                                                                                                          startVelocity,
                                                                                                                                          endVelocity,
                                                                                                                                          maximumVelocity,
                                                                                                                                          maximumAcceleration);

            return timed_trajectory;
        }

        bool DriveMotionPlanner::isDone(void)
        {
            return mCurrentTrajectory != NULL && mCurrentTrajectory->isDone();
        }

        void DriveMotionPlanner::reset(void)
        {
            mError = const_cast<Pose2d *>(&Pose2d::identity());
            mOutput = ChassisSpeeds();
            mLastTime = ck::math::POS_INF_F;
        }

        void DriveMotionPlanner::setFollowerType(FollowerType type)
        {
            mFollowerType = type;
        }

        void DriveMotionPlanner::setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory)
        {
            *mCurrentTrajectory = trajectory;
            // *mSetpoint = trajectory.getState();

            for (int i = 0; i < trajectory.trajectory().length(); ++i)
            {
                if (trajectory.trajectory().getState(i).velocity() > ck::math::kEpsilon)
                {
                    mIsReversed = false;
                    break;
                }
                else if (trajectory.trajectory().getState(i).velocity() < -ck::math::kEpsilon)
                {
                    mIsReversed = true;
                    break;
                }
            }
        }

        // protected DriveOutput updateRamsete(DifferentialDrive.DriveDynamics dynamics, Pose2d current_state) {
        //     // Implements eqn. 5.12 from https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
        //     final double kBeta = 1.5;  // >0.
        //     final double kZeta = 0.7;  // Damping coefficient, [0, 1].

        //     // Compute gain parameter.
        //     final double k = 2.0 * kZeta * Math.sqrt(kBeta * dynamics.chassis_velocity.linear * dynamics.chassis_velocity
        //             .linear + dynamics.chassis_velocity.angular * dynamics.chassis_velocity.angular);

        //     // Compute error components.
        //     final double angle_error_rads = mError.getRotation().getRadians();
        //     final double sin_x_over_x = Util.epsilonEquals(angle_error_rads, 0.0, 1E-2) ?
        //             1.0 : mError.getRotation().sin() / angle_error_rads;
        //     final DifferentialDrive.ChassisState adjusted_velocity = new DifferentialDrive.ChassisState(
        //             dynamics.chassis_velocity.linear * mError.getRotation().cos() +
        //                     k * Units.inches_to_meters(mError.getTranslation().x()),
        //             dynamics.chassis_velocity.angular + k * angle_error_rads +
        //                     dynamics.chassis_velocity.linear * kBeta * sin_x_over_x * Units.inches_to_meters(mError
        //                             .getTranslation().y()));

        //     // Compute adjusted left and right wheel velocities.
        //     dynamics.chassis_velocity = adjusted_velocity;
        //     dynamics.wheel_velocity = mModel.solveInverseKinematics(adjusted_velocity);

        //     dynamics.chassis_acceleration.linear = mDt == 0 ? 0.0 : (dynamics.chassis_velocity.linear - prev_velocity_
        //             .linear) / mDt;
        //     dynamics.chassis_acceleration.angular = mDt == 0 ? 0.0 : (dynamics.chassis_velocity.angular - prev_velocity_
        //             .angular) / mDt;

        //     prev_velocity_ = dynamics.chassis_velocity;

        //     DifferentialDrive.WheelState feedforward_voltages = mModel.solveInverseDynamics(dynamics.chassis_velocity,
        //             dynamics.chassis_acceleration).voltage;

        //     return new DriveOutput(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics.wheel_acceleration
        //             .left, dynamics.wheel_acceleration.right, feedforward_voltages.left, feedforward_voltages.right);
        // }

        Output DriveMotionPlanner::updateRamsete(ck::physics::DriveDynamics dynamics)
        {
            (void)dynamics;
            return Output();
        }

        // public DriveOutput update(double timestamp, Pose2d current_state) {
        //     if (mCurrentTrajectory == null) return new DriveOutput();

        //     if (mCurrentTrajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime)) {
        //         mLastTime = timestamp;
        //     }

        //     mDt = timestamp - mLastTime;
        //     mLastTime = timestamp;
        //     TrajectorySamplePoint<TimedState<Pose2dWithCurvature>> sample_point = mCurrentTrajectory.advance(mDt);
        //     mSetpoint = sample_point.state();
        //     if (!mCurrentTrajectory.isDone()) {
        //         // Generate feedforward voltages.
        //         final double velocity_m = Units.inches_to_meters(mSetpoint.velocity());
        //         final double curvature_m = Units.meters_to_inches(mSetpoint.state().getCurvature());
        //         final double dcurvature_ds_m = Units.meters_to_inches(Units.meters_to_inches(mSetpoint.state()
        //                 .getDCurvatureDs()));
        //         final double acceleration_m = Units.inches_to_meters(mSetpoint.acceleration());
        //         final DifferentialDrive.DriveDynamics dynamics = mModel.solveInverseDynamics(
        //                 new DifferentialDrive.ChassisState(velocity_m, velocity_m * curvature_m),
        //                 new DifferentialDrive.ChassisState(acceleration_m,
        //                         acceleration_m * curvature_m + velocity_m * velocity_m * dcurvature_ds_m));
        //         mError = current_state.inverse().transformBy(mSetpoint.state().getPose());
        //         if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {
        //             mOutput = new DriveOutput(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics
        //                     .wheel_acceleration.left, dynamics.wheel_acceleration.right, dynamics.voltage
        //                     .left, dynamics.voltage.right);
        //         } else if (mFollowerType == FollowerType.PURE_PURSUIT) {
        //             mOutput = updatePurePursuit(dynamics, current_state);
        //         } else if (mFollowerType == FollowerType.PID) {
        //             mOutput = updatePID(dynamics, current_state);
        //         } else if (mFollowerType == FollowerType.RAMSETE) {
        //             mOutput = updateRamsete(dynamics, current_state);
        //         }
        //     } else {
        //         // TODO Possibly switch to a pose stabilizing controller?
        //         mOutput = new DriveOutput();
        //     }
        //     return mOutput;
        // }
        Output* DriveMotionPlanner::update(double timestamp, team254_geometry::Pose2d current_state)
        {
            (void)timestamp;
            (void)current_state;

            Output* out = new Output();
            return out;
        }

    } // namespace planners
} // namespace ck