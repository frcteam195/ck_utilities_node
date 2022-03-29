#pragma once

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace ck
{
    namespace math
    {
        void get_rpy_from_quaternion(tf2::Quaternion q, double (&rpy_out)[3])
        {
            tf2::Matrix3x3 m(q);
            m.getRPY(rpy_out[0], rpy_out[1], rpy_out[2]);
        }

        double get_yaw_from_quaternion(tf2::Quaternion q)
        {
            double rpy[3] = {};
            get_rpy_from_quaternion(q, rpy);
            return rpy[2];
        }

        double get_pitch_from_quaternion(tf2::Quaternion q)
        {
            double rpy[3] = {};
            get_rpy_from_quaternion(q, rpy);
            return rpy[1];
        }

        double get_roll_from_quaternion(tf2::Quaternion q)
        {
            double rpy[3] = {};
            get_rpy_from_quaternion(q, rpy);
            return rpy[0];
        }
    }
};