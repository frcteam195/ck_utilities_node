#pragma once

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include "Eigen/Dense"
namespace geometry
{
    class Pose; class Transform; class Twist;

    typedef Eigen::Vector3f Translation;

    class Rotation : public Eigen::Vector3f
    {
    public:
        Rotation();
        Rotation(const Eigen::Vector3f& other);
        Rotation operator*(const float &other);
        Rotation operator=(const Eigen::Vector3f &other);
        float roll();
        void roll(float value);
        float pitch();
        void pitch(float value);
        float yaw();
        void yaw(float value);
    };

    class Pose
    {
    public:
        Pose()
        {
            this->position.setZero();
            this->orientation.setIdentity();
        }
        Pose twist(Twist twist_, double time_s);
        Pose transform(Transform transform_);
        Transform get_Transform(Pose pose_);
        Translation position;
        Rotation orientation;
    };

    class Transform
    {
    public:
        Transform()
        {
            this->linear.setZero();
            this->angular.setIdentity();
        }
        Transform rotate(Rotation rotation);
        Rotation get_Rotation_To();
        Translation linear;
        Rotation angular;
    };

    class  Twist
    {
    public:
        Twist()
        {
            this->linear.setZero();
            this->angular.setIdentity();
        }
        Translation linear;
        Rotation angular;
    };

}

std::ostream& operator<<(std::ostream& os, const geometry::Pose& value);
std::ostream& operator<<(std::ostream& os, const geometry::Twist& value);
std::ostream& operator<<(std::ostream& os, const geometry::Transform& value);