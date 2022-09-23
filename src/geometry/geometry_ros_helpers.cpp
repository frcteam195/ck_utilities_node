#include "ck_utilities/geometry/geometry_ros_helpers.hpp"

geometry::Rotation geometry::to_rotation(tf2::Quaternion q)
{
    geometry::Rotation result;
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    result.roll(roll);
    result.pitch(pitch);
    result.yaw(yaw);
    return result;
}

geometry::Rotation geometry::to_rotation(geometry_msgs::Quaternion q)
{
    tf2::Quaternion q2;
    tf2::fromMsg(q, q2);
    return to_rotation(q2);
}

geometry::Rotation geometry::to_rotation(tf2::Vector3 v)
{
    geometry::Rotation result;
    result.roll(v.x());
    result.pitch(v.y());
    result.yaw(v.z());
    return result;
}

geometry::Rotation geometry::to_rotation(geometry_msgs::Vector3 v)
{
    tf2::Vector3 v2;
    tf2::fromMsg(v, v2);
    return to_rotation(v2);
}

geometry::Translation geometry::to_translation(tf2::Vector3 v)
{
    geometry::Translation result;
    result[0] = v.x();
    result[1] = v.y();
    result[2] = v.z();
    return result;
}

geometry::Translation geometry::to_translation(geometry_msgs::Vector3 v)
{
    tf2::Vector3 v2;
    tf2::fromMsg(v, v2);
    return to_translation(v2);
}

geometry::Translation geometry::to_translation(geometry_msgs::Point p)
{
    tf2::Vector3 v;
    tf2::fromMsg(p, v);
    return to_translation(v);
}

geometry::Pose geometry::to_pose(geometry_msgs::Pose p)
{
    geometry::Pose result;
    result.position = to_translation(p.position);
    result.orientation = to_rotation(p.orientation);
    return result;
}

geometry::Transform geometry::to_transform(geometry_msgs::Transform t)
{
    geometry::Transform result;
    result.linear = to_translation(t.translation);
    result.angular = to_rotation(t.rotation);
    return result;
}

geometry::Twist geometry::to_twist(geometry_msgs::Twist t)
{
    geometry::Twist result;
    result.linear = to_translation(t.linear);
    result.angular = to_rotation(t.angular);
    return result;
}
