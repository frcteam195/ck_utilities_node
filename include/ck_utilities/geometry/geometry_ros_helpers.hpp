#include "ck_utilities/geometry/geometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_eigen/tf2_eigen.h"
#include "nav_msgs/Odometry.h"

namespace geometry
{
    geometry::Rotation to_rotation(tf2::Quaternion q);
    geometry::Rotation to_rotation(geometry_msgs::Quaternion q);
    geometry::Rotation to_rotation(tf2::Vector3 v);
    geometry::Rotation to_rotation(geometry_msgs::Vector3 v);
    geometry::Translation to_translation(tf2::Vector3 v);
    geometry::Translation to_translation(geometry_msgs::Vector3 v);
    geometry::Translation to_translation(geometry_msgs::Point p);
    geometry::Pose to_pose(geometry_msgs::Pose p);
    geometry::Transform to_transform(geometry_msgs::Transform t);
    geometry::Twist to_twist(geometry_msgs::Twist t);
    geometry::Covariance to_covariance(boost::array<double, 36UL> &c);
    tf2::Quaternion to_tf2_quat(geometry::Rotation r);
    geometry_msgs::Quaternion to_msg_quat(geometry::Rotation r);
    tf2::Vector3 to_tf2(geometry::Translation t);
    tf2::Transform to_tf2(geometry::Transform t);
    tf2::Transform to_tf2(geometry::Transform t);
    geometry_msgs::Transform to_msg(geometry::Transform t);
    geometry_msgs::Point to_msg_point(geometry::Translation t);
    tf2::Vector3 to_tf2(geometry::Rotation r);
    geometry_msgs::Vector3 to_msg(geometry::Rotation r);
    geometry_msgs::Vector3 to_msg(geometry::Translation t);
    geometry_msgs::Pose to_msg(geometry::Pose p);
    geometry_msgs::Twist to_msg(geometry::Twist t);
    geometry_msgs::Transform to_msg(geometry::Transform t);
    boost::array<double, 36UL> to_msg(geometry::Covariance &c);
}