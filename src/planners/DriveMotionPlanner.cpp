#include "ck_utilities/planners/DriveMotionPlanner.hpp"

#include "ck_utilities/physics/DCMotorTransmission.hpp"
#include "ck_utilities/trajectory/TrajectoryUtil.hpp"
#include "ck_utilities/trajectory/timing/CentripetalAccelerationConstraint.hpp"
#include "ck_utilities/trajectory/timing/DifferentialDriveDynamicsConstraint.hpp"
#include "ck_utilities/trajectory/timing/TimingUtil.hpp"
#include "ck_utilities/trajectory/timing/TimingConstraint.hpp"

#include "ck_utilities/Logger.hpp"

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>


double smallest_traversal(double angle, double target_angle)
{
    double left = -ck::math::normalize_to_2_pi(angle - target_angle);
    double right = ck::math::normalize_to_2_pi(target_angle - angle);
    if(fabs(left) < fabs(right))
    {
        return left;
    }
    return right;
}

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

        DriveMotionPlanner::DriveMotionPlanner(double max_translational_velocity)
            : kMaxVelocityMetersPerSecond(max_translational_velocity),
              mSpeedLookahead(kAdaptivePathMinLookaheadDistance, kAdaptivePathMaxLookaheadDistance, 0.0, math::meters_to_inches(kMaxVelocityMetersPerSecond))
        {}

        void DriveMotionPlanner::setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory)
        {
            *mCurrentTrajectory = trajectory;
            *mPathSetpoint = trajectory.getState();
            *mHeadingSetpoint = trajectory.getHeading();
            *mLastHeadingSetpoint = trajectory.getHeading();
            lastProgress = 0.0;
            mLastPathSetpoint = nullptr;
            useDefaultCook = true;
            mCurrentTrajectoryLength = mCurrentTrajectory->trajectory().getLastPoint().state_.t();
            for (int i = 0; i < trajectory.trajectory().length(); i++)
            {
                if (trajectory.trajectory().getState(i).velocity() > math::kEpsilon)
                {
                    mIsReversed = false;
                }
                else if (trajectory.trajectory().getState(i).velocity() < -math::kEpsilon)
                {
                    mIsReversed = true;
                    break;
                }
            }
        }

        void DriveMotionPlanner::reset(void)
        {
            *mTranslationError = Translation2d::identity();
            *mHeadingError = Rotation2d::identity();
            // mLastHeadingSetpoint = nullptr;
            mLastPathSetpoint = nullptr;
            *mOutput = ChassisSpeeds();
            mLastTime = math::POS_INF;
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

        ChassisSpeeds DriveMotionPlanner::updatePurePursuit(Pose2d current_state, double feedforwardOmegaRadiansPerSecond)
        {
            double lookahead_time = kPathLookaheadTime;
            double kLookaheadSearchDt = 0.01;
            TimedState<Pose2dWithCurvature> lookahead_state = mCurrentTrajectory->preview(lookahead_time).state();
            double actual_lookahead_distance = mPathSetpoint->state().distance(lookahead_state.state());
            double adaptive_lookahead_distance = mSpeedLookahead.getLookaheadForSpeed(mPathSetpoint->velocity()) + kAdaptiveErrorLookaheadCoefficient * mError->getTranslation().norm();

            while (actual_lookahead_distance < adaptive_lookahead_distance &&
                   mCurrentTrajectory->getRemainingProgress() > lookahead_time)
            {
                lookahead_time += kLookaheadSearchDt;
                lookahead_state = mCurrentTrajectory->preview(lookahead_time).state();
                actual_lookahead_distance = mPathSetpoint->state().distance(lookahead_state.state());
            }

            if (actual_lookahead_distance < adaptive_lookahead_distance)
            {
                Translation2d translation((mIsReversed ? -1.0 : 1.0) * (kPathMinLookaheadDistance - actual_lookahead_distance), 0.0);
                Pose2d transform = Pose2d::fromTranslation(translation);
                Pose2dWithCurvature pose(lookahead_state.state().getPose().transformBy(transform), 0.0);
                lookahead_state = TimedState<Pose2dWithCurvature>(pose, lookahead_state.t(), lookahead_state.velocity(), lookahead_state.acceleration());
            }

            Translation2d lookaheadTranslation(current_state.getTranslation(), lookahead_state.state().getTranslation());

            static tf2_ros::TransformBroadcaster tfBroadcaster;

            geometry_msgs::TransformStamped lookahead;
            lookahead.header.frame_id = "unaligned_base_link";
            lookahead.header.stamp = ros::Time().now();
            lookahead.child_frame_id = "traj_look";
            lookahead.transform.translation.x = math::inches_to_meters(lookaheadTranslation.x());
            lookahead.transform.translation.y = math::inches_to_meters(lookaheadTranslation.y());
            lookahead.transform.translation.z = 0;
            lookahead.transform.rotation.x = 0;
            lookahead.transform.rotation.y = 0;
            lookahead.transform.rotation.z = 0;
            lookahead.transform.rotation.w = 1;

            tfBroadcaster.sendTransform(lookahead);

            Rotation2d steeringDirection = lookaheadTranslation.direction().rotateBy(current_state.inverse().getRotation());

            double normalizedSpeed = std::abs(mPathSetpoint->velocity()) / math::meters_to_inches(kMaxVelocityMetersPerSecond);

            useDefaultCook = true;

            if (normalizedSpeed > defaultCook)// || mPathSetpoint->t() > (mCurrentTrajectoryLength / 2.0))
            {
                useDefaultCook = false;
            }
            if (useDefaultCook)
            {
                normalizedSpeed = defaultCook;
            }

            Translation2d steeringVector(steeringDirection.cos() * normalizedSpeed, steeringDirection.sin() * normalizedSpeed);
            ChassisSpeeds chassisSpeeds(steeringVector.x() * kMaxVelocityMetersPerSecond,
                                        steeringVector.y() * kMaxVelocityMetersPerSecond,
                                        feedforwardOmegaRadiansPerSecond);

            double kPathKTheta = 0.3;

            chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond + kPathKTheta * mError->getRotation().getRadians();

            return chassisSpeeds;
        }

        ChassisSpeeds DriveMotionPlanner::update(double timestamp, Pose2d current_state)
        {
            if (mCurrentTrajectory == nullptr) return ChassisSpeeds();

            if (mCurrentTrajectory->getProgress() == 0.0 && !std::isfinite(mLastTime))
            {
                mLastTime = timestamp;
                lastProgress = 0.0;

                *mInitialHeading = Rotation2d(mCurrentTrajectory->trajectory().getHeading(0).state());
                Rotation2d finalHeading = mCurrentTrajectory->trajectory().getLastPoint().heading_.state();
                mTotalTime = mCurrentTrajectory->trajectory().getLastPoint().state_.t() -
                             mCurrentTrajectory->trajectory().getState(0).t();
                *mRotationDiff = finalHeading.rotateBy(mInitialHeading->inverse());

                if (mRotationDiff->getRadians() > math::PI)
                {
                    *mRotationDiff = mRotationDiff->inverse().rotateBy(Rotation2d::fromRadians(math::PI));
                }

                mStartTime = timestamp;
                if (std::abs(mRotationDiff->getRadians()) < 0.1)
                {
                    mDTheta = 0.0;
                }
                else
                {
                    mDTheta = mRotationDiff->getRadians() / mTotalTime;
                }
            }

            mDt = timestamp - mLastTime;
            mLastTime = timestamp;
            TrajectorySamplePoint<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> sample_point;

            // *mHeadingSetpoint = TimedState<Rotation2d>(mInitialHeading->rotateBy(mRotationDiff->times(math::min(1.0, (timestamp - mStartTime) / mTotalTime))));

            *mHeadingSetpoint = mCurrentTrajectory->getHeading();

            if (!math::epsilonEquals(mHeadingSetpoint->state().getRadians(), mLastHeadingSetpoint->state().getRadians(), 0.001) && mCurrentTrajectory->getProgress() > lastProgress)
            {
                if (math::epsilonEquals(mDt, 0.0, 0.0001))
                {
                    mDTheta = 0;
                }
                else
                {
                    static bool firstDTheta = true;
                    if (firstDTheta)
                    {
                        mDTheta = 0;
                        firstDTheta = false;
                    }
                    else
                    {
                        float angle_delta = smallest_traversal(mLastHeadingSetpoint->state().getRadians(), mHeadingSetpoint->state().getRadians());
                        mDTheta = angle_delta / (mTotalTime / mCurrentTrajectory->trajectory().length());
                    }
                    // std::cout << mCurrentTrajectory->getProgress() << ", ";
                    // std::cout << mHeadingSetpoint->state().getRadians() << ", " << mLastHeadingSetpoint->state().getRadians();
                    // std::cout << ", dtheta: " << mDTheta << std::endl;
                }
                *mLastHeadingSetpoint = *mHeadingSetpoint;
                lastProgress = mCurrentTrajectory->getProgress();
            }

            *mCurrentState = current_state;

            if (!isDone())
            {
                sample_point = mCurrentTrajectory->advance(mDt);
                *mError = current_state.inverse().transformBy(mPathSetpoint->state().getPose());
                *mError = Pose2d(mError->getTranslation(), current_state.getRotation().inverse().rotateBy(mHeadingSetpoint->state().getRotation()));

                if (mFollowerType == FollowerType::PURE_PURSUIT)
                {
                    double searchStepSize = 1.0;
                    double previewQuantity = 0.0;
                    double searchDirection = 1.0;
                    double forwardDistance = distance(current_state, previewQuantity + searchStepSize);
                    double reverseDistance = distance(current_state, previewQuantity - searchStepSize);
                    searchDirection = math::signum(reverseDistance - forwardDistance);

                    while (searchStepSize > 0.001)
                    {
                        if (math::epsilonEquals(distance(current_state, previewQuantity), 0.0, 0.01)) break;

                        while(distance(current_state, previewQuantity + searchStepSize * searchDirection) < distance(current_state, previewQuantity))
                        {
                            previewQuantity += searchStepSize * searchDirection;
                        }
                        searchStepSize /= 10.0;
                        searchDirection *= -1;
                    }
                    sample_point = mCurrentTrajectory->advance(previewQuantity);
                    *mPathSetpoint = sample_point.state();

                    *mOutput = updatePurePursuit(current_state, mDTheta);
                }
            }
            else
            {
                *mOutput = ChassisSpeeds();
            }

            return *mOutput;
        }

        bool DriveMotionPlanner::isDone(void)
        {
            return mCurrentTrajectory != NULL && mCurrentTrajectory->isDone();
        }

        Translation2d DriveMotionPlanner::getTranslationalError()
        {
            return Translation2d(math::inches_to_meters(mError->getTranslation().x()),
                                 math::inches_to_meters(mError->getTranslation().y()));
        }

        Rotation2d DriveMotionPlanner::getHeadingError()
        {
            return mError->getRotation();
        }

        double DriveMotionPlanner::distance(Pose2d current_state, double additional_progress)
        {
            return mCurrentTrajectory->preview(additional_progress).state().state().getPose().distance(current_state);
        }

        TimedState<Pose2dWithCurvature> DriveMotionPlanner::getPathSetpoint()
        {
            return *mPathSetpoint;
        }

        TimedState<Rotation2d> DriveMotionPlanner::getHeadingSetpoint()
        {
            return *mHeadingSetpoint;
        }

        void DriveMotionPlanner::setFollowerType(FollowerType type)
        {
            mFollowerType = type;
        }

    } // namespace planners
} // namespace ck