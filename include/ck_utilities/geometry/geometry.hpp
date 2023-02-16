#pragma once

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include "Eigen/Dense"
namespace geometry
{
    class Pose; class Transform; class Twist;

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

    class Translation : public Eigen::Vector3f
    {
    public:
        Translation();
        Translation(const Eigen::Vector3f& other);
        Translation operator*(const float &other);
        Translation operator=(const Eigen::Vector3f &other);
        using Eigen::Vector3f::x;
        void x(float value);
        using Eigen::Vector3f::y;
        void y(float value);
        using Eigen::Vector3f::z;
        void z(float value);
        Translation Rotate(Rotation rotation);
    };

    class Pose
    {
    public:
        Pose() { }
        Pose twist(Twist twist_, double time_s);
        Pose transform(Transform transform_);
        Transform get_Transform(Pose pose_);
        Translation position;
        Rotation orientation;
        Pose operator+(const Pose &other);
        Pose operator/(const float &other);
    };

    class Transform
    {
    public:
        Transform() { }
        Transform rotate(Rotation rotation);
        Rotation get_Rotation_To();
        Translation linear;
        Rotation angular;
    };

    class  Twist
    {
    public:
        Twist() { }
        Twist rotate(Rotation rotation);
        Translation linear;
        Rotation angular;
    };

    class Covariance : Eigen::Matrix<float, 6, 6>
    {
        public:
        Covariance();
        float x_var();
        void x_var(float var);
        float y_var();
        void y_var(float var);
        float z_var();
        void z_var(float var);
        float roll_var();
        void roll_var(float var);
        float yaw_var();
        void yaw_var(float var);
        float pitch_var();
        void pitch_var(float var);
        using Eigen::DenseCoeffsBase<Eigen::Matrix<float, 6, 6>, 1>::operator();
    };

}

std::ostream& operator<<(std::ostream& os, const geometry::Pose& value);
std::ostream& operator<<(std::ostream& os, const geometry::Twist& value);
std::ostream& operator<<(std::ostream& os, const geometry::Transform& value);