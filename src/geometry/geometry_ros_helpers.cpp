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

geometry::Covariance geometry::to_covariance(boost::array<double, 36UL> &c)
{
    geometry::Covariance new_covariance;
    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            new_covariance(i, j) = c[i*6+j];
        }
    }
    return new_covariance;
}

tf2::Quaternion geometry::to_tf2_quat(geometry::Rotation r)
{
    tf2::Quaternion result;
    result.setRPY(r.roll(), r.pitch(), r.yaw());
    return result;
}

geometry_msgs::Quaternion geometry::to_msg_quat(geometry::Rotation r)
{
    return tf2::toMsg(geometry::to_tf2_quat(r));
}

tf2::Vector3 geometry::to_tf2(geometry::Translation t)
{
    tf2::Vector3 result;
    result.setX(t.x());
    result.setY(t.y());
    result.setZ(t.z());
    return result;
}

tf2::Transform geometry::to_tf2(geometry::Transform t)
{
    tf2::Transform result;
    result.setOrigin(geometry::to_tf2(t.linear));
    result.setRotation(geometry::to_tf2_quat(t.angular));
    return result;
}

geometry_msgs::Transform geometry::to_msg(geometry::Transform t)
{
    geometry_msgs::Transform result;
    result.translation = to_msg(t.linear);
    result.rotation = to_msg_quat(t.angular);
    return result;
}

geometry_msgs::Point geometry::to_msg_point(geometry::Translation t)
{
    tf2::Vector3 t2 = to_tf2(t);
    geometry_msgs::Point p;
    p.x = t2.x();
    p.y = t2.y();
    p.z = t2.z();
    return p;
}

tf2::Vector3 geometry::to_tf2(geometry::Rotation r)
{
    tf2::Vector3 result;
    result.setX(r.roll());
    result.setY(r.pitch());
    result.setZ(r.yaw());
    return result;
}

geometry_msgs::Vector3 geometry::to_msg(geometry::Rotation r)
{
    return tf2::toMsg(to_tf2(r));
}

geometry_msgs::Vector3 geometry::to_msg(geometry::Translation t)
{
    return tf2::toMsg(to_tf2(t));
}

geometry_msgs::Pose geometry::to_msg(geometry::Pose p)
{
    geometry_msgs::Pose result;
    result.orientation = geometry::to_msg_quat(p.orientation);
    result.position = geometry::to_msg_point(p.position);
    return result;
}

geometry_msgs::Twist geometry::to_msg(geometry::Twist t)
{
    geometry_msgs::Twist result;
    result.angular = geometry::to_msg(t.angular);
    result.linear = geometry::to_msg(t.linear);
    return result;
}

boost::array<double, 36UL> to_msg(geometry::Covariance &c)
{
    boost::array<double, 36UL> output;
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            output[i*6+j] = c(i,j);
        }
    }
    return output;
}