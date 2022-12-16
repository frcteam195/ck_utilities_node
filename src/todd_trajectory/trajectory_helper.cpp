#include "ck_utilities/todd_trajectory/trajectory_helpers.hpp"
#include "ck_utilities/CKMath.hpp"

using geometry::Pose;
using geometry::Transform;

float calculate_desired_track (Pose starting_pose, Pose ending_pose)
{
    return starting_pose.get_Transform(ending_pose).get_Rotation_To().yaw();
}

float calculate_along_track_distance(Pose starting_pose, Pose ending_pose, float desired_track)
{
    Transform t =  starting_pose.get_Transform(ending_pose);
    double sign_pspe = ck::math::signum(desired_track) == ck::math::signum(starting_pose.orientation.yaw()) ? 1 : -1;
    double pspe = sign_pspe * t.linear.norm();
    double theta_pspe = t.angular.yaw();
    double theta_dtk = desired_track;
    double theta_atk = theta_pspe - theta_dtk;
    double b_prime = pspe * cos(theta_atk);
    return b_prime;
}

float calculate_track_angle_error(Pose starting_pose, Pose ending_pose, float desired_track)
{
    Transform t =  starting_pose.get_Transform(ending_pose);
    double theta_pspe = t.angular.yaw();
    double theta_dtk = desired_track;
    double theta_atk = theta_pspe - theta_dtk;
    return theta_atk;
}

float calculate_cross_track_distance(Pose starting_pose, Pose ending_pose, float desired_track)
{
    Transform t =  starting_pose.get_Transform(ending_pose);
    double sign_pspe = ck::math::signum(desired_track) == ck::math::signum(starting_pose.orientation.yaw()) ? 1 : -1;
    double pspe = sign_pspe * t.linear.norm();
    double theta_pspe = t.angular.yaw();
    double theta_dtk = desired_track;
    double theta_atk = theta_pspe - theta_dtk;
    double b_prime = pspe * sin(theta_atk);
    return b_prime;
}