#pragma once

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include "Eigen/Dense"
namespace geometry
{
    class Pose; class Transform; class Twist;

    typedef Eigen::Vector3f Rotation;
    typedef Eigen::Vector3f Translation;

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