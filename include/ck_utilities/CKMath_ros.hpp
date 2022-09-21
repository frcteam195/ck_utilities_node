#pragma once

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

namespace ck
{
    namespace math
    {
        inline void get_rpy_from_quaternion(tf2::Quaternion q, double (&rpy_out)[3])
        {
            tf2::Matrix3x3 m(q);
            m.getRPY(rpy_out[0], rpy_out[1], rpy_out[2]);
        }

        inline double get_yaw_from_quaternion(tf2::Quaternion q)
        {
            double rpy[3] = {};
            get_rpy_from_quaternion(q, rpy);
            return rpy[2];
        }

        inline double get_pitch_from_quaternion(tf2::Quaternion q)
        {
            double rpy[3] = {};
            get_rpy_from_quaternion(q, rpy);
            return rpy[1];
        }

        inline double get_roll_from_quaternion(tf2::Quaternion q)
        {
            double rpy[3] = {};
            get_rpy_from_quaternion(q, rpy);
            return rpy[0];
        }

        inline geometry_msgs::Pose null_pose()
        {
            geometry_msgs::Pose result;
            result.position.x = 0;
            result.position.y = 0;
            result.position.z = 0;
            tf2::Quaternion t;
            t.setRPY(0,0,0);
            result.orientation.w = t.getW();
            result.orientation.x = t.getX();
            result.orientation.y = t.getY();
            result.orientation.z = t.getZ();
            return result;
        }

        inline geometry_msgs::Pose apply_twist(geometry_msgs::Pose initial_pose, geometry_msgs::Twist twist, double dt)
        {
            geometry_msgs::Pose result = initial_pose;
            result.position.x += twist.linear.x * dt;
            result.position.y += twist.linear.y * dt;
            result.position.z += twist.linear.z * dt;
            tf2::Quaternion applied_quat;
            applied_quat.setRPY(twist.angular.x * dt, twist.angular.y * dt, twist.angular.z * dt);
            tf2::Quaternion initial_quat;
            tf2::fromMsg(initial_pose.orientation, initial_quat);
            tf2::Quaternion overall_quat = initial_quat * applied_quat;
            result.orientation = tf2::toMsg(overall_quat);
            return result;
        }

        inline geometry_msgs::Pose apply_transform(geometry_msgs::Pose initial_pose, geometry_msgs::Transform transform)
        {
            geometry_msgs::Pose result;
            result.position.x = initial_pose.position.x + transform.translation.x;
            result.position.y = initial_pose.position.y + transform.translation.y;
            result.position.z = initial_pose.position.z + transform.translation.z;

            tf2::Quaternion initial_quat;
            tf2::Quaternion rotation_quat;
            initial_quat.setW(initial_pose.orientation.w);
            initial_quat.setX(initial_pose.orientation.x);
            initial_quat.setY(initial_pose.orientation.y);
            initial_quat.setZ(initial_pose.orientation.z);
            rotation_quat.setW(transform.rotation.w);
            rotation_quat.setX(transform.rotation.x);
            rotation_quat.setY(transform.rotation.y);
            rotation_quat.setZ(transform.rotation.z);

            tf2::Quaternion result_rotation = initial_quat * rotation_quat;
            result.orientation.w = result_rotation.getW();
            result.orientation.x = result_rotation.getX();
            result.orientation.y = result_rotation.getY();
            result.orientation.z = result_rotation.getZ();

            return result;
        }
    }
};