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
        {
            mDt = 0.0;

            double wheelRadiusMeters = math::inches_to_meters(K_DRIVE_WHEEL_RADIUS_INCHES);
            double wheelRadiusSquaredMeters = wheelRadiusMeters * wheelRadiusMeters;
            double torquePerVolt = wheelRadiusSquaredMeters * K_ROBOT_LINEAR_INERTIA / (2.0 * K_DRIVE_KA);

            physics::DCMotorTransmission transmission = physics::DCMotorTransmission(1.0 / K_DRIVE_KV, torquePerVolt, K_DRIVE_V_INTERCEPT);

            double effectiveWheelbaseRadius = math::inches_to_meters(K_DRIVE_WHEEL_TRACK_WIDTH_INCHES / 2.0 * K_TRACK_SCRUB_FACTOR);

            mModel = new physics::DifferentialDrive(K_ROBOT_LINEAR_INERTIA,
                                                    K_ROBOT_ANGULAR_INERTIA,
                                                    K_ROBOT_ANGULAR_DRAG,
                                                    wheelRadiusMeters,
                                                    effectiveWheelbaseRadius,
                                                    transmission, transmission);
        }

        trajectory::Trajectory<trajectory::timing::TimedState<team254_geometry::Pose2dWithCurvature>> DriveMotionPlanner::generateTrajectory(bool reversed,
                                                                                                                                     std::vector<team254_geometry::Pose2d> waypoints,
                                                                                                                                     double maximumVelocity,     // Inches per Second
                                                                                                                                     double maximumAcceleration, // Inches per Second^2
                                                                                                                                     double maximumVoltage)
        {
            return this->generateTrajectory(reversed,
                                            waypoints,
                                            0.0,
                                            0.0,
                                            maximumVelocity,
                                            maximumAcceleration,
                                            maximumVoltage);
        }

        trajectory::Trajectory<trajectory::timing::TimedState<team254_geometry::Pose2dWithCurvature>> DriveMotionPlanner::generateTrajectory(bool reversed,
                                                                                                                                     std::vector<team254_geometry::Pose2d> waypoints,
                                                                                                                                     double startVelocity,       // Inches per Second
                                                                                                                                     double endVelocity,         // Inches per Second
                                                                                                                                     double maximumVelocity,     // Inches per Second
                                                                                                                                     double maximumAcceleration, // Inches per Second^2
                                                                                                                                     double maximumVoltage)
        {
            std::vector<team254_geometry::Pose2d> modifiedWaypoints = waypoints;
            team254_geometry::Pose2d flipFactor = team254_geometry::Pose2d::fromRotation(team254_geometry::Rotation2d(-1.0, 0.0, false));

            // Flip the waypoints if the trajectory is reversed.
            if (reversed)
            {
                for (size_t i = 0; i < waypoints.size(); ++i)
                {
                    modifiedWaypoints.push_back(waypoints[i].transformBy(flipFactor));
                }
            }

            // Create a trajectory from the splines.
            trajectory::Trajectory<team254_geometry::Pose2dWithCurvature> trajectory = trajectory::TrajectoryUtil::trajectoryFromSplineWaypoints(modifiedWaypoints, kMaxDx, kMaxDy, kMaxDtheta);

            if (reversed)
            {
                std::vector<team254_geometry::Pose2dWithCurvature> flippedCurvaturePoints;

                for (int i = 0; i < trajectory.length(); ++i)
                {
                    flippedCurvaturePoints.push_back(team254_geometry::Pose2dWithCurvature(trajectory.getState(i).getPose().transformBy(flipFactor),
                                                                                   -trajectory.getState(i).getCurvature(),
                                                                                   trajectory.getState(i).getDCurvatureDs()));
                }

                trajectory = trajectory::Trajectory<team254_geometry::Pose2dWithCurvature>(flippedCurvaturePoints);
            }

            // Create the constraint that the robot must be able to traverse the trajectory without ever applying more than the specified voltage.
            trajectory::timing::DifferentialDriveDynamicsConstraint<team254_geometry::Pose2dWithCurvature> driveConstraints(*this->mModel, maximumVoltage);
            trajectory::timing::CentripetalAccelerationConstraint centripetalAccelConstraint(kMaxCentripetalAccel);

            std::vector<trajectory::timing::TimingConstraint<team254_geometry::Pose2dWithCurvature> *> constraints {&driveConstraints, &centripetalAccelConstraint};

            trajectory::DistanceView<team254_geometry::Pose2dWithCurvature> distanceView(trajectory);
            trajectory::Trajectory<trajectory::timing::TimedState<team254_geometry::Pose2dWithCurvature>> timedTrajectory = trajectory::timing::TimingUtil::timeParameterizeTrajectory(reversed,
                                                                                                                                                                               distanceView,
                                                                                                                                                                               kMaxDx,
                                                                                                                                                                               constraints,
                                                                                                                                                                               startVelocity,
                                                                                                                                                                               endVelocity,
                                                                                                                                                                               maximumVelocity,
                                                                                                                                                                               maximumAcceleration);

            return timedTrajectory;
        }

        bool DriveMotionPlanner::isDone(void)
        {
            return mCurrentTrajectory != NULL && mCurrentTrajectory->isDone();
        }

        void DriveMotionPlanner::reset(void)
        {
            mError = team254_geometry::Pose2d::identity();
            mOutput = new Output();
            mLastTime = ck::math::POS_INF_F;
        }

        void DriveMotionPlanner::setFollowerType(FollowerType type)
        {
            mFollowerType = type;
        }

        void DriveMotionPlanner::setTrajectory(trajectory::TrajectoryIterator<trajectory::timing::TimedState<team254_geometry::Pose2dWithCurvature>> trajectory)
        {
            *mCurrentTrajectory = trajectory;
            *mSetpoint = trajectory.getState();

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
            const double kBeta = 1.5;
            const double kZeta = 0.7;

            const double k = 2.0 * kZeta * std::sqrt(kBeta * dynamics.chassis_velocity.linear * dynamics.chassis_velocity.linear
                             + dynamics.chassis_velocity.angular * dynamics.chassis_velocity.angular);

            const double angular_error_rads = mError.getRotation().getRadians();
            const double sin_x_over_x = ck::math::epsilonEquals(angular_error_rads, 0.0, 1e-2) ?
                                        1.0 : mError.getRotation().sin() / angular_error_rads;
            physics::ChassisState adjusted_velocity;
            adjusted_velocity.linear = dynamics.chassis_velocity.linear * mError.getRotation().cos() +
                                       k * ck::math::inches_to_meters(mError.getTranslation().x());
            adjusted_velocity.angular = dynamics.chassis_velocity.angular + k * angular_error_rads +
                                        dynamics.chassis_velocity.linear * kBeta * sin_x_over_x *
                                        ck::math::inches_to_meters(mError.getTranslation().y());

            dynamics.chassis_velocity = adjusted_velocity;
            dynamics.wheel_velocity = mModel->solveInverseKinematics(adjusted_velocity);

            dynamics.chassis_acceleration.linear = mDt == 0 ? 0.0 : (dynamics.chassis_velocity.linear -
                                                                    prev_velocity_.linear) / mDt;
            dynamics.chassis_acceleration.angular = mDt == 0 ? 0.0 : (dynamics.chassis_velocity.angular -
                                                                      prev_velocity_.angular) / mDt;

            prev_velocity_ = dynamics.chassis_velocity;

            physics::WheelState feedforward_voltages = mModel->solveInverseDynamics(dynamics.chassis_velocity, dynamics.chassis_acceleration).voltage;

            Output result(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics.wheel_acceleration.left,
                          dynamics.wheel_acceleration.right, feedforward_voltages.left, feedforward_voltages.right);

            return result;
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
            if (mCurrentTrajectory == nullptr)
            {
                mOutput->setZeros();
                return mOutput;
            }

            if (mCurrentTrajectory->getProgress() == 0.0 && !std::isinf(mLastTime))
            {
                mLastTime = timestamp;
            }

            mDt = timestamp - mLastTime;
            mLastTime = timestamp;
            trajectory::TrajectorySamplePoint<trajectory::timing::TimedState<team254_geometry::Pose2dWithCurvature>> sample_point = mCurrentTrajectory->advance(mDt);
            // mSetpoint = &(sample_point.state());

            if (!mCurrentTrajectory->isDone())
            {
                double velocity_m = math::inches_to_meters(mSetpoint->velocity());
                double curvature_m = math::meters_to_inches(mSetpoint->state().getCurvature());
                double dcurvature_ds_m = math::meters_to_inches(math::meters_to_inches(mSetpoint->state().getDCurvatureDs()));
                double acceleration_m = math::inches_to_meters(mSetpoint->acceleration());

                physics::ChassisState chassis_velocity{velocity_m, velocity_m * curvature_m};
                physics::ChassisState chassis_acceleration{acceleration_m, acceleration_m * curvature_m + velocity_m * velocity_m * dcurvature_ds_m};

                const physics::DriveDynamics dynamics = mModel->solveInverseDynamics(chassis_velocity, chassis_acceleration);
                mError = current_state.inverse().transformBy(mSetpoint->state().getPose());
                if (mFollowerType == FollowerType::NONLINEAR_FEEDBACK)
                {
                    // mOutput = &updateRamsete(dynamics);
                    updateRamsete(dynamics);
                    mOutput->setZeros();
                }
            }
            else
            {
                mOutput->setZeros();
            }

            return mOutput;
        }

    } // namespace planners
} // namespace ck