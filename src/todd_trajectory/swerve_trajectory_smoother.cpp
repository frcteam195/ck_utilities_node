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
            DetailedTrajectoryPoint new_pose = last_point;
            new_pose.associated_base_point = basic_point - smoothed_path.base_path.points.begin();
            new_pose.target_pose = (*basic_point).pose;
            new_pose.desired_track = desired_track;
            new_pose.desired_speed = (*basic_point).speed;
            new_pose = this->project(new_pose);
            smoothed_path.points.push_back(new_pose);
            along_track_distance = calculate_along_track_distance(new_pose.pose, (*basic_point).pose, desired_track);
        }
    }
}

float p_ctrl(float error, float p_gain, float output_max, float output_min)
{
    return std::max(output_min, std::min(error * p_gain, output_max));
}

DetailedTrajectoryPoint SwerveTrajectorySmoother::project(DetailedTrajectoryPoint initial_pose)
{
    const static float cross_track_gain = 0.1;
    const static float cross_track_max = 10;
    const static float cross_track_min = -10;
    const static float track_angle_error_gain = 0.05;
    const static float track_angle_error_max = 5;
    const static float track_angle_error_min = -5;

    float cross_track_error = -calculate_cross_track_distance(initial_pose.pose, initial_pose.target_pose, initial_pose.desired_track);
    float track_angle_error = calculate_track_angle_error(initial_pose.pose, initial_pose.target_pose, initial_pose.desired_track);

    float cross_track_error_pid_result = p_ctrl(cross_track_error, cross_track_gain, cross_track_max, cross_track_min);
    float track_angle_pid_result = p_ctrl(track_angle_error, track_angle_error_gain, track_angle_error_max, track_angle_error_min);
    float pid_result = (cross_track_error_pid_result + track_angle_pid_result) / 2.0f;


    
}