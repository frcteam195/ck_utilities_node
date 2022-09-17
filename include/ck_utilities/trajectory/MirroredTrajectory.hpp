#pragma once

#include "ck_utilities/geometry/Pose2dWithCurvature.hpp"
#include "ck_utilities/trajectory/Trajectory.hpp"
#include "ck_utilities/trajectory/timing/TimedState.hpp"

namespace ck
{
    namespace trajectory
    {
        class MirroredTrajectory
        {
        public:
            MirroredTrajectory(trajectory::Trajectory<trajectory::timing::TimedState<geometry::Pose2dWithCurvature> > right);

            trajectory::Trajectory<trajectory::timing::TimedState<geometry::Pose2dWithCurvature> > get(bool left = false);

            trajectory::Trajectory<trajectory::timing::TimedState<geometry::Pose2dWithCurvature> > left;
            trajectory::Trajectory<trajectory::timing::TimedState<geometry::Pose2dWithCurvature> > right;
        };
    } // namespace trajectory
} // namespace ck
