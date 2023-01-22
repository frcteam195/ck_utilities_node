#pragma once

// TODO: Move a lot of these constants to robot params.
namespace ck
{
    // constexpr double K_LOOPER_DT = 0.01;
    // constexpr double K_LOG_REPORT_RATE = 0.02;

    // constexpr double  K_DRIVE_WHEEL_TRACK_WIDTH_INCHES = 23.5;
    // constexpr double  K_DRIVE_WHEEL_DIAMETER_INCHES = 5.0 * 0.9625;
    // constexpr double  K_DRIVE_WHEEL_RADIUS_INCHES = K_DRIVE_WHEEL_DIAMETER_INCHES / 2.0;
    // constexpr double K_TRACK_SCRUB_FACTOR = 1.0; // Tune me!

    // constexpr double K_ROBOT_LINEAR_INERTIA = 68.946;   // Kilograms
    // constexpr double K_ROBOT_ANGULAR_INERTIA = 125;     // Kilograms * Meters^2
    // constexpr double K_ROBOT_ANGULAR_DRAG = 0.1;        // Newtonmeters per Radians per Second
    // constexpr double K_DRIVE_V_INTERCEPT = 0.30165000;  // 0.781046438 Angular Volts
    // constexpr double K_DRIVE_KV = 0.186163041;          // Volts per Radians per Second
    // constexpr double K_DRIVE_KA = 0.0086739979;         // Volts per Radians per Second^2

    // constexpr double K_LIDAR_X_OFFSET = -3.3211;
    // constexpr double K_LIDAR_Y_OFFSET = 0.0;
    // constexpr double K_LIDAR_YAW_ANGLE_DEGREES = 0.0;
    constexpr double K_LOOPER_DT = 0.01;
    constexpr double K_LOG_REPORT_RATE = 0.02;

    constexpr double  K_DRIVE_WHEEL_TRACK_WIDTH_INCHES = 25.625;
    // constexpr double  K_DRIVE_WHEEL_DIAMETER_INCHES = 4.0025;
    constexpr double  K_DRIVE_WHEEL_DIAMETER_INCHES = 3.9;
    // constexpr double  K_DRIVE_WHEEL_DIAMETER_INCHES = 4.00;
    constexpr double  K_DRIVE_WHEEL_RADIUS_INCHES = K_DRIVE_WHEEL_DIAMETER_INCHES / 2.0;
    constexpr double K_TRACK_SCRUB_FACTOR = 1.0; // Tune me!

    constexpr double K_ROBOT_LINEAR_INERTIA = 68.946;   // Kilograms
    constexpr double K_ROBOT_ANGULAR_INERTIA = 6.087973483894603;     // Kilograms * Meters^2
    constexpr double K_ROBOT_ANGULAR_DRAG = 15;        // Newtonmeters per Radians per Second
    constexpr double K_DRIVE_V_INTERCEPT = 0.378582;  // 0.781046438 Angular Volts
    constexpr double K_DRIVE_KV = 0.22754012375000002;          // Volts per Radians per Second
    constexpr double K_DRIVE_KA = 0.018831762500000012;         // Volts per Radians per Second^2

    constexpr double K_LIDAR_X_OFFSET = -3.3211;
    constexpr double K_LIDAR_Y_OFFSET = 0.0;
    constexpr double K_LIDAR_YAW_ANGLE_DEGREES = 0.0;

    constexpr double kPathLookaheadTime = 0.25;
    constexpr double kPathMinLookaheadDistance = 12.0;
    constexpr double kAdaptivePathMinLookaheadDistance = 6.0;
    constexpr double kAdaptivePathMaxLookaheadDistance = 24.0;
    constexpr double kAdaptiveErrorLookaheadCoefficient = 0.01;
    constexpr double kMaxVelocityMetersPerSecond = 3; // from da poofs (calculate this for our robot)
} // namespace ck