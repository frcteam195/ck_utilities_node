#include "ck_utilities/trajectory/MirroredTrajectory.hpp"

#include "ck_utilities/trajectory/TrajectoryUtil.hpp"

namespace ck
{
    namespace trajectory
    {
        MirroredTrajectory::MirroredTrajectory(trajectory::Trajectory<trajectory::timing::TimedState<geometry::Pose2dWithCurvature>> right) : right(right)
        {
            this->left = trajectory::TrajectoryUtil::mirrorTimed(right);
        }

        trajectory::Trajectory<trajectory::timing::TimedState<geometry::Pose2dWithCurvature>> MirroredTrajectory::get(bool left)
        {
            return left ? this->left : this->right;
        }
    } // namespace trajectory
} // namespace ck
