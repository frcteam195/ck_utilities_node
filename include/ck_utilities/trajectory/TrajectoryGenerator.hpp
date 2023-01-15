#pragma once

#include "ck_utilities/Singleton.hpp"
#include "ck_utilities/planners/DriveMotionPlanner.hpp"
#include "ck_utilities/trajectory/MirroredTrajectory.hpp"

namespace ck
{
    namespace trajectory
    {
        class TrajectorySet
        {
        public:
            // MirroredTrajectory *test90DegPath;

            TrajectorySet(void);
        };

        class TrajectoryGenerator : public Singleton<TrajectoryGenerator>
        {
            friend Singleton;

        private:
            // static const double kMaxVoltage = 9.0;
            // static const double kMaxAccel = 40.0;
            // static const double kMaxVelocity = 60.0;

            // static const double kFirstPathMaxVoltage = 9.0;
            // static const double kFirstPathMaxAccel = 80.0;
            // static const double kFirstPathMaxVel = 100.0;

            // static const double kMaxCentripetalAccel = 100.0;

            planners::DriveMotionPlanner *mMotionPlanner = NULL;
            TrajectorySet *mTrajectorySet = NULL;

            TrajectoryGenerator(void);

        public:
            // MirroredTrajectory generateMirroredTrajectory(bool reversed,
            //                                               std::vector<team254_geometry::Pose2d> waypoints,
            //                                               /* TODO: Implement Timing Constraint, */
            //                                               double maximumVelocity,
            //                                               double maximumAcceleration,
            //                                               double maximumVoltage);

            void generateTrajectories(void);

            TrajectorySet *getTrajectorySet(void);
        };

    } // namespace trajectory
} // namespace ck