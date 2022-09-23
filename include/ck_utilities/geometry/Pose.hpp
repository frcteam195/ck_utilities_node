#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include "Eigen/Dense"

class Pose
{
public:
    Eigen::Vector3f position;
    // Eigen::Quaternion<float> orientation;
};

class Transform
{
public:
    Eigen::Vector3f linear;
    // Eigen::Quaternion<float> angular;
};

typedef Transform Twist;



