#include "ck_utilities/geometry/geometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_eigen/tf2_eigen.h"

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
}