#include "ck_utilities/todd_trajectory/swerve_trajectory_smoother.hpp"
#include "ck_utilities/todd_trajectory/trajectory_helpers.hpp"

using SwerveTrajectory::DetailedTrajectory;
using SwerveTrajectory::DetailedTrajectoryPoint;
using SwerveTrajectory::BasicTrajectory;
using SwerveTrajectory::BasicTrajectoryPoint;
using SwerveTrajectory::SwerveTrajectorySmoother;

DetailedTrajectory SwerveTrajectorySmoother::smooth_path
    (BasicTrajectory trajectory)
{
    DetailedTrajectory smoothed_path;
    smoothed_path.base_path = trajectory;

    if (smoothed_path.base_path.points.empty())
        return smoothed_path;

    DetailedTrajectoryPoint initial_pose(*(smoothed_path.base_path.points.begin()));
    smoothed_path.points.push_back(initial_pose);

    if (smoothed_path.base_path.points.size() == 1)
        return smoothed_path;

    BasicTrajectoryPoint last_point = *(smoothed_path.base_path.points.begin());
    DetailedTrajectoryPoint detailed_last_point(last_point);

    for (auto basic_point = smoothed_path.base_path.points.begin() + 1;
         basic_point != smoothed_path.base_path.points.end();
         basic_point++)
    {
        float desired_track = calculate_desired_track(last_point.pose, (*basic_point).pose);
        float along_track_distance = calculate_along_track_distance(detailed_last_point.pose, (*basic_point).pose, desired_track);
        while (along_track_distance > 0)
        {

        }
    }
}