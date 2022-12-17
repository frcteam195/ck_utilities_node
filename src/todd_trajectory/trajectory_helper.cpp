#include "ck_utilities/todd_trajectory/trajectory_helpers.hpp"
#include "ck_utilities/CKMath.hpp"
#include <iostream>

using geometry::Pose;
using geometry::Transform;

float smallest_acute_angle(float a, float b)
{
    float ab = ck::math::normalize_to_2_pi(a-b);
    float ba = ck::math::normalize_to_2_pi(b-a);
    return std::min(ab, ba);
}

float calculate_desired_track (Pose starting_pose, Pose ending_pose)
{
    return starting_pose.get_Transform(ending_pose).get_Rotation_To().yaw();
}

float calculate_along_track_distance(Pose starting_pose, Pose ending_pose, float desired_track)
{
    Transform t =  ending_pose.get_Transform(starting_pose);
    return t.linear.norm() * cos(smallest_acute_angle((M_PI + desired_track), t.get_Rotation_To().yaw()));
}

float calculate_track_angle_error(Pose starting_pose, Pose ending_pose, float desired_track)
{
    (void) ending_pose;
    return smallest_acute_angle(desired_track, starting_pose.orientation.yaw());
}

float calculate_cross_track_distance(Pose starting_pose, Pose ending_pose, float desired_track)
{
    Transform t =  ending_pose.get_Transform(starting_pose);
    return t.linear.norm() * sin(smallest_acute_angle((M_PI + desired_track), t.get_Rotation_To().yaw()));
}