#pragma once

#include "ck_utilities/CKMath.hpp"
#include "ck_utilities/Constants.hpp"
#include "ck_utilities/team254_geometry/Geometry.hpp"
#include "ck_utilities/physics/DifferentialDrive.hpp"
#include "ck_utilities/trajectory/Lookahead.hpp"
#include "ck_utilities/trajectory/Trajectory.hpp"
#include "ck_utilities/trajectory/TrajectoryIterator.hpp"
#include "ck_utilities/trajectory/timing/TimingConstraint.hpp"
#include "ck_utilities/trajectory/timing/TimedState.hpp"

using namespace ck::trajectory;
using namespace ck::trajectory::timing;
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

            double vxMetersPerSecond;
            double vyMetersPerSecond;
            double omegaRadiansPerSecond;
        };

        class Output
        {
        public:
            Output() {}

            Output(double leftVelocity,
                   double rightVelocity,
                   double leftAcceleration,
                   double rightAcceleration,
                   double leftFeedForwardVoltage,
                   double rightFeedForwardVoltage);

            double mLeftVelocity;  // Radians per Second
            double mRightVelocity; // Radians per Second

            double mLeftAcceleration;  // Radians per Second^2
            double mRightAcceleration; // Radians per Second^2

            double mLeftFeedForwardVoltage;
            double mRightFeedForwardVoltage;

            void flip(void);

            void setZeros(void);
        };

        class DriveMotionPlanner
        {
        public:
            enum FollowerType
            {
                FEEDFORWARD_ONLY,
                PURE_PURSUIT,
                PID,
            };

            DriveMotionPlanner(void);

            Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> generateTrajectory(bool reversed,
                                                                                                   std::vector<Pose2d> waypoints,
                                                                                                   std::vector<Rotation2d> headings,
                                                                                                   double maximumVelocity,   // Inches per Second
                                                                                                   double maximumAcceleration, // Inches per Second^2
                                                                                                   double maximumVoltage);
                                                                                                                     
            Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> generateTrajectory(bool reversed,
                                                                                                   std::vector<Pose2d> waypoints,
                                                                                                   std::vector<Rotation2d> headings,
                                                                                                   double startVelocity, // Inches per Second
                                                                                                   double endVelocity, // Inches per Second
                                                                                                   double maximumVelocity, // Inches per Second
                                                                                                   double maximumAcceleration, // Inches per Second^2
                                                                                                   double maximumVoltage);

            bool isDone(void);

            void reset(void);

            void setFollowerType(FollowerType type);

            void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory);

            Output updateRamsete(ck::physics::DriveDynamics dynamics);

            Output* update(double timestamp, team254_geometry::Pose2d current_state);

        private:
            static constexpr double kMaxDx = 2.0;
            static constexpr double kMaxDy = 0.25;
            // static constexpr double kMaxDtheta = 0.0872665; // 5 Degrees
            double kMaxDTheta = math::deg2rad(1.0);
            static constexpr double kMaxCentripetalAccel = 60.0;

            FollowerType mFollowerType = FollowerType::PURE_PURSUIT;

            double defaultCook = 0.4;
            bool useDefaultCook = true;

            TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> *mCurrentTrajectory = nullptr;

            bool mIsReversed = false;
            double mLastTime = math::POS_INF;

            TimedState<Pose2dWithCurvature> *mLastPathSetpoint = nullptr;
            TimedState<Pose2dWithCurvature> *mPaetSetpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature::identity());
            TimedState<Rotation2d> *mHeadingSetpoint = nullptr;
            TimedState<Rotation2d> *mLastHeadingSetpoint = new TimedState<Rotation2d>(Rotation2d::identity());

            Pose2d *mError = new Pose2d(Pose2d::identity());

            Translation2d *mTranslationError = new Translation2d(Translation2d::identity());
            Rotation2d *mHeadingError = new Rotation2d(Rotation2d::identity());
            Rotation2d *mInitialHeading = new Rotation2d(Rotation2d::identity());
            Rotation2d *mRotationDiff = new Rotation2d(Rotation2d::identity());

            Pose2d *mCurrentState = new Pose2d(Pose2d::identity());

            double mCurrentTrajectoryLength = 0.0;
            double mTotalTime = math::POS_INF;
            double mStartTime = math::POS_INF;
            double mDTheta = 0.0;

            ChassisSpeeds mOutput;

            Lookahead mSpeedLookahead;

            double mDt = 0.0;

        };

    } // namespace planners
} // namespace ck