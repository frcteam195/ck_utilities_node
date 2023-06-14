#include "ck_utilities/team254_swerve/SwerveSetpointGenerator.hpp"
#include "ck_utilities/team254_geometry/Geometry.hpp"
#include "ck_utilities/planners/ChassisSpeeds.hpp"

#include "ros/ros.h"

#include <gtest/gtest.h>
#include <thread>
#include <vector>
#include <iostream>

using namespace ck::team254_swerve;
using namespace ck::team254_geometry;
using namespace ck::planners;

extern ros::NodeHandle *node;

constexpr double kRobotSide = 0.616; // m
constexpr double kDt = 0.01; // s
double kMaxSteeringVelocityError = ck::math::deg2rad(2.0); // rad/s
constexpr double kMaxAccelerationError = 0.1; // m/s^2

KinematicLimits kKinematicLimits
{
    5.0,    // m/s
    10.0,   //m/s2
    ck::math::deg2rad(1500.0)   //rad/s
};

SwerveDriveKinematics kKinematics(std::vector<Translation2d> {
    // Front left
    Translation2d(kRobotSide / 2.0, kRobotSide / 2.0),
    // Front right
    Translation2d(kRobotSide / 2.0, -kRobotSide / 2.0),
    // Back left
    Translation2d(-kRobotSide / 2.0, kRobotSide / 2.0),
    // Back right
    Translation2d(-kRobotSide / 2.0, -kRobotSide / 2.0)
});


void SatisfiesConstraints(SwerveSetpoint prev, SwerveSetpoint next) {
    for (size_t i = 0; i < prev.mModuleStates.size(); ++i) {
        SwerveModuleState prevModule = prev.mModuleStates[i];
        SwerveModuleState nextModule = next.mModuleStates[i];
        Rotation2d diffRotation = prevModule.angle.inverse().rotateBy(nextModule.angle);
        ASSERT_TRUE(std::abs(diffRotation.getRadians()) < kKinematicLimits.kMaxSteeringVelocity + kMaxSteeringVelocityError);
        ASSERT_TRUE(std::abs(nextModule.speedMetersPerSecond) <= kKinematicLimits.kMaxDriveVelocity);
        ASSERT_TRUE(std::abs(nextModule.speedMetersPerSecond - prevModule.speedMetersPerSecond) / kDt <= kKinematicLimits.kMaxDriveAcceleration + kMaxAccelerationError);
    }
}

SwerveSetpoint driveToGoal(SwerveSetpoint prevSetpoint, ChassisSpeeds goal, SwerveSetpointGenerator generator) {
    std::cout << "Driving to goal state " << goal << std::endl;
    std::cout << "Initial state: " << prevSetpoint << std::endl;;
    while (!prevSetpoint.mChassisSpeeds.toTwist2d().epsilonEquals(goal.toTwist2d(), ck::math::kEpsilon)) {
        SwerveSetpoint newsetpoint = generator.generateSetpoint(kKinematicLimits, prevSetpoint, goal, kDt);
        std::cout << (newsetpoint) << std::endl;
        SatisfiesConstraints(prevSetpoint, newsetpoint);
        prevSetpoint = newsetpoint;
    }
    return prevSetpoint;
}


TEST(CKUtilitiesTest, SwerveSetpointGeneratorTest)
{
    std::vector<SwerveModuleState> initialStates {
        SwerveModuleState(),
        SwerveModuleState(),
        SwerveModuleState(),
        SwerveModuleState()
    };
    SwerveSetpoint setpoint(ChassisSpeeds(), initialStates);

    SwerveSetpointGenerator generator(kKinematics);

    ChassisSpeeds goalSpeeds(0.0, 0.0, 1.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = ChassisSpeeds(0.0, 0.0, -1.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = ChassisSpeeds(0.0, 0.0, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = ChassisSpeeds(1.0, 0.0, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = ChassisSpeeds(0.0, 1.0, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = ChassisSpeeds(0.1, -1.0, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = ChassisSpeeds(1.0, -0.5, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);

    goalSpeeds = ChassisSpeeds(1.0, 0.4, 0.0);
    setpoint = driveToGoal(setpoint, goalSpeeds, generator);
}



