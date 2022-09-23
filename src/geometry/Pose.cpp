#include "Pose.hpp"

using namespace geometry;

static Eigen::Quaternionf quaternion_from_rotation(Rotation rotation)
{
    Eigen::AngleAxisd rollAngle(rotation.x(), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(rotation.y(), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(rotation.z(), Eigen::Vector3d::UnitX());

    Eigen::Quaternionf q = rollAngle * yawAngle * pitchAngle;

    return q;
}

static Rotation rotation_from_Quaternion(Eigen::Quaternionf q)
{
    return q.matrix().eulerAngles(0, 1, 2);
}

Pose Pose::twist(Twist twist_, double time_s)
{
    Pose result = *this;
    result.position += (twist_.linear * time_s);
    result.orientation = rotation_from_Quaternion(quaternion_from_rotation(result.orientation) * quaternion_from_rotation(twist_.angular * time_s));
    return result;
}

Pose Pose::transform(Transform transform_)
{
    Pose result = *this;
    result.position += transform_.linear;
    result.orientation = rotation_from_Quaternion(quaternion_from_rotation(result.orientation) * quaternion_from_rotation(transform_.angular));
    return result;
}

Transform Pose::get_Transform(Pose pose_)
{
    Transform result;
    result.linear = pose_.position - this->position;
    result.angular = rotation_from_Quaternion(quaternion_from_rotation(pose_.orientation) * quaternion_from_rotation(this->orientation).inverse());
    return result;
}

Transform Transform::rotate(Rotation rotation)
{
    Transform result = *this;

    Eigen::Quaternionf q = quaternion_from_rotation(rotation);
    result.linear = q * result.linear;
    result.angular = result.angular * q;
    return result;
}