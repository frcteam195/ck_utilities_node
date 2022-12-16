#include "ck_utilities/todd_trajectory/swerve_trajectory_smoother.hpp"
#include "ck_utilities/todd_trajectory/trajectory_helpers.hpp"
#include "ck_utilities/CKMath.hpp"
#include <iostream>

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
    std::cout << "Gonna go actually loop" << std::endl;

    for (auto basic_point = smoothed_path.base_path.points.begin() + 1;
         basic_point != smoothed_path.base_path.points.end();
         basic_point++)
    {
        std::cout << "Here in my outer loop" << std::endl;
        float desired_track = calculate_desired_track(last_point.pose, (*basic_point).pose);
        float along_track_distance = calculate_along_track_distance(detailed_last_point.pose, (*basic_point).pose, desired_track);
        while (along_track_distance > 0)
        {
            std::cout << "Here in my inner loop" << std::endl;
            DetailedTrajectoryPoint new_pose = detailed_last_point;
            new_pose.associated_base_point = basic_point - smoothed_path.base_path.points.begin();
            new_pose.target_pose = (*basic_point).pose;
            new_pose.desired_track = desired_track;
            new_pose.desired_speed = (*basic_point).speed;
            new_pose = this->project(new_pose);
            smoothed_path.points.push_back(new_pose);
            detailed_last_point = new_pose;
            along_track_distance = calculate_along_track_distance(new_pose.pose, (*basic_point).pose, desired_track);
            std::cout << "Got my new along track distance: " << along_track_distance << std::endl;
            std::cout << "With my pose: " << new_pose.pose.position << std::endl;
            std::cout << "Target Pose: " << new_pose.target_pose << std::endl;
            std::cout << "DTK: " << new_pose.desired_track << std::endl;
            std::cout << "SPD: " << new_pose.desired_speed << std::endl;
        }
    }

    return smoothed_path;
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

    //Project point
    //mult accel by timestep and add to speed coming in
    DetailedTrajectoryPoint dtp(initial_pose);

    if (initial_pose.desired_speed > initial_pose.speed && initial_pose.desired_speed >= 0)
    {
        //accelerating
        dtp.speed += config.track_acceleration_by_speed.lookup(initial_pose.speed) * config.time_step_seconds;
        dtp.speed = std::min(dtp.speed, initial_pose.desired_speed);
    }
    else if(initial_pose.desired_speed < initial_pose.speed && initial_pose.desired_speed >= 0)
    {
        //decelerating
        dtp.speed -= config.track_deceleration_by_speed.lookup(initial_pose.speed) * config.time_step_seconds;
        dtp.speed = std::max(dtp.speed, initial_pose.desired_speed);
    }
    else if (initial_pose.desired_speed < initial_pose.speed && initial_pose.desired_speed <= 0)
    {
        //accelerating
        dtp.speed -= config.track_acceleration_by_speed.lookup(initial_pose.speed) * config.time_step_seconds;
        dtp.speed = std::max(dtp.speed, initial_pose.desired_speed);
    }
    else if (initial_pose.desired_speed > initial_pose.speed && initial_pose.desired_speed <= 0)
    {
        //decelerating
        dtp.speed += config.track_deceleration_by_speed.lookup(initial_pose.speed) * config.time_step_seconds;
        dtp.speed = std::min(dtp.speed, initial_pose.desired_speed);
    }
    else
    {
        dtp.speed = initial_pose.desired_speed;
    }

    //Look at initial pose orientation
    Pose p(dtp.pose);

    float constrained_pid_result = ck::math::signum(pid_result) * std::min(std::abs(pid_result), std::abs(config.track_turn_rate_by_speed.lookup((dtp.speed + initial_pose.speed) / 2.0f)));

    float new_desired_track_angle = initial_pose.desired_track + constrained_pid_result;
    dtp.pose.orientation.yaw(new_desired_track_angle);

    //sin(newyaw) * speed * time -----> add to pose x
    //cos(newyaw) * speed * time -----> add to pose y

    float old_x = dtp.pose.position.x();
    float old_y = dtp.pose.position.y();

    dtp.pose.position.x(old_x + (sin(dtp.pose.orientation.yaw()) * dtp.speed * config.time_step_seconds));
    dtp.pose.position.y(old_y + (cos(dtp.pose.orientation.yaw()) * dtp.speed * config.time_step_seconds));

    dtp.pose = p;
    return dtp;
}