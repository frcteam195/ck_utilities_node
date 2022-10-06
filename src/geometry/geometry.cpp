#include "ck_utilities/geometry/geometry.hpp"

using namespace geometry;

static Eigen::Quaternionf quaternion_from_rotation(Rotation rotation)
{
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(rotation.x(), Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(rotation.y(), Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(rotation.z(), Eigen::Vector3f::UnitZ());

    return q;
}

static Rotation rotation_from_quaternion(Eigen::Quaternionf q)
{
    return (Rotation) q.matrix().eulerAngles(0, 1, 2);
}

Rotation::Rotation()
{
    this->setZero();
}

Rotation::Rotation(const Eigen::Vector3f& other)
{
    (*this)[0] = other.x();
    (*this)[1] = other.y();
    (*this)[2] = other.z();
}

Rotation Rotation::operator*(const float &other)
{
    return (Rotation) Eigen::Vector3f::operator*(other);
}

Rotation Rotation::operator=(const Eigen::Vector3f &other)
{
    Rotation temp(other);
    *this = temp;
    return *this;
}

float Rotation::roll()
{
    return this->x();
}

void Rotation::roll(float value)
{
    (*this)[0] = value;
}

float Rotation::pitch()
{
    return this->y();
}

void Rotation::pitch(float value)
{
    (*this)[1] = value;
}

float Rotation::yaw()
{
    return this->z();
}

void Rotation::yaw(float value)
{
    (*this)[2] = value;
}

Pose Pose::twist(Twist twist_, double time_s)
{
    Pose result = *this;
    result.position += (twist_.linear * time_s);
    result.orientation = rotation_from_quaternion(quaternion_from_rotation(result.orientation) * quaternion_from_rotation(twist_.angular * time_s));
    return result;
}

Pose Pose::transform(Transform transform_)
{
    Pose result = *this;
    result.position += transform_.linear;
    result.orientation = rotation_from_quaternion(quaternion_from_rotation(result.orientation) * quaternion_from_rotation(transform_.angular));
    return result;
}

Transform Pose::get_Transform(Pose pose_)
{
    Transform result;
    result.linear = pose_.position - this->position;
    result.angular = rotation_from_quaternion(quaternion_from_rotation(pose_.orientation) * quaternion_from_rotation(this->orientation).inverse());
    return result;
}

Transform Transform::rotate(Rotation rotation)
{
    Transform result = *this;

    Eigen::Quaternionf q = quaternion_from_rotation(rotation);
    result.linear = q * this->linear;

    Eigen::Quaternionf base = quaternion_from_rotation(this->angular);
    result.angular = rotation_from_quaternion(base * q);
    return result;
}

Rotation Transform::get_Rotation_To()
{
    Rotation result;

    float x_y_hypot = std::sqrt((this->linear.x() * this->linear.x()) + (this->linear.y() * this->linear.y()));
    result.roll(0);
    result.pitch(std::asin(this->linear.norm() / x_y_hypot));
    result.yaw(std::asin(x_y_hypot / this->linear.x()));
    return result;
}

std::ostream& operator<<(std::ostream& os, const geometry::Pose& value)
{
    std::stringstream s;
    s << "Position:" << value.position << std::endl;
    s << "Orientation:" << value.orientation << std::endl;
    os << s.str();
    return os;
}

std::ostream& operator<<(std::ostream& os, const geometry::Twist& value)
{
    std::stringstream s;
    s << "Linear:" << value.linear << std::endl;
    s << "Angular:" << value.angular << std::endl;
    os << s.str();
    return os;
}

std::ostream& operator<<(std::ostream& os, const geometry::Transform& value)
{
    std::stringstream s;
    s << "Translation:" << value.linear << std::endl;
    s << "Rotation: " << value.angular << std::endl;
    os << s.str();
    return os;
}