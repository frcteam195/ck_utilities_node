#include "ck_utilities/geometry/geometry.hpp"
#include "ros/ros.h"

using namespace geometry;

static Eigen::Vector3f rotate_vector_by_quaternion(const Eigen::Vector3f& v, const Eigen::Quaternionf& q)
{
    Eigen::Vector3f vprime;
    // Extract the vector part of the quaternion
    Eigen::Vector3f u(q.x(), q.y(), q.z());

    // Extract the scalar part of the quaternion
    float s = q.w();

    // Do the math
    vprime = 2.0f * u.dot(v) * u
          + (s*s - u.dot(u)) * v
          + 2.0f * s * u.cross(v);

    return vprime;
}

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

Translation::Translation()
{
    this->setZero();
}

Translation::Translation(const Eigen::Vector3f& other)
{
    (*this)[0] = other.x();
    (*this)[1] = other.y();
    (*this)[2] = other.z();
}

Translation Translation::operator*(const float &other)
{
    return (Rotation) Eigen::Vector3f::operator*(other);
}

Translation Translation::operator=(const Eigen::Vector3f &other)
{
    Translation temp(other);
    *this = temp;
    return *this;
}

void Translation::x(float value)
{
    (*this)[0] = value;
}

void Translation::y(float value)
{
    (*this)[1] = value;
}

void Translation::z(float value)
{
    (*this)[2] = value;
}

Translation Translation::Rotate(Rotation rotation)
{
    return rotate_vector_by_quaternion(*this, quaternion_from_rotation(rotation));
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
    result.position += rotate_vector_by_quaternion(transform_.linear ,quaternion_from_rotation(this->orientation));
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
    result.setZero();
    float x_y_hypot = std::sqrt((this->linear.x() * this->linear.x()) + (this->linear.y() * this->linear.y()));
    result.roll(0);
    result.pitch(std::atan2(this->linear.z(), x_y_hypot));
    result.yaw(std::atan2(this->linear.y(), this->linear.x()));
    return result;
}

geometry::Covariance::Covariance()
{
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            (*this)(i,j) = 0.0;
        }
    }
}

float geometry::Covariance::x_var()
{
    return  (*this)(0,0);
}

void geometry::Covariance::x_var(float var)
{
    (*this)(0,0) = var;
}

float geometry::Covariance::y_var()
{
    return  (*this)(1,1);
}

void geometry::Covariance::y_var(float var)
{
    (*this)(1,1) = var;
}

float geometry::Covariance::z_var()
{
    return  (*this)(2,2);
}

void geometry::Covariance::z_var(float var)
{
    (*this)(2,2) = var;
}

float geometry::Covariance::roll_var()
{
    return  (*this)(3,3);
}

void geometry::Covariance::roll_var(float var)
{
    (*this)(3,3) = var;
}

float geometry::Covariance::yaw_var()
{
    return  (*this)(4,4);
}

void geometry::Covariance::yaw_var(float var)
{
    (*this)(4,4) = var;
}

float geometry::Covariance::pitch_var()
{
    return  (*this)(5,5);
}

void geometry::Covariance::pitch_var(float var)
{
    (*this)(5,5) = var;
}

std::ostream& operator<<(std::ostream& os, const geometry::Pose& value)
{
    std::stringstream s;
    s << "Position: " << value.position.x() << " " << value.position.y() << " " << value.position.z() << " ";
    s << "Orientation: " << value.orientation.x() << " " << value.orientation.y() << " " << value.orientation.z() << std::endl;
    os << s.str();
    return os;
}

std::ostream& operator<<(std::ostream& os, const geometry::Twist& value)
{
    std::stringstream s;
    s << "Linear: " << value.linear.x() << " " << value.linear.y() << " " << value.linear.z() << " ";
    s << "Angular: " << value.angular.x() << " " << value.angular.y() << " " << value.angular.z() << std::endl;
    os << s.str();
    return os;
}

std::ostream& operator<<(std::ostream& os, const geometry::Transform& value)
{
    std::stringstream s;
    s << "Translation: " << value.linear.x() << " " << value.linear.y() << " " << value.linear.z() << " ";
    s << "Rotation: " << value.angular.x() << " " << value.angular.y() << " " << value.angular.z() << std::endl;
    os << s.str();
    return os;
}
