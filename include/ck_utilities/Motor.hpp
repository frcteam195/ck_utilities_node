#pragma once
#if __has_include("ros/ros.h")
#include <cstdint>
#include "ck_ros_base_msgs_node/Motor_Control.h"
#include "ck_ros_base_msgs_node/Motor_Configuration.h"
#include <atomic>
#include <mutex>

class MotorMaster;

class MotorData
{
public:
    uint8_t motor_id;
    ck_ros_base_msgs_node::Motor_Config motor_config;
    uint8_t master_id;
};

class MotorConfig
{
private:
    uint8_t motor_id;
    MotorData active_config;
    MotorData pending_config;
public:

    enum class NeutralMode
    {
        COAST=1,
        BRAKE=2,
    };

    enum class LimitSwitchSource
    {
        FeedbackConnector = 0,
        RemoteTalon = 1,
        RemoteTalonSRX = 1,
        RemoteCANifier = 2,
        Deactivated = 3
    };

    enum class LimitSwitchNormal
    {
        NormallyOpen = 0,
        NormallyClosed = 1,
        Disabled = 2
    };

    void apply();
    void set_fast_master(bool enable);
    void set_kP(double value);
    void set_kI(double value);
    void set_kD(double value);
    void set_kF(double value);
    void set_kP_Slot1(double value);
    void set_kI_Slot1(double value);
    void set_kD_Slot1(double value);
    void set_kF_Slot1(double value);
    void set_active_gain_slot(int8_t slotIdx);
    void set_i_zone(double value);
    void set_max_i_accum(double value);
    void set_allowed_closed_loop_error(double value);
    void set_max_closed_loop_peak_output(double value);
    void set_motion_cruise_velocity(double value);
    void set_motion_acceleration(double value);
    void set_motion_s_curve_strength(int32_t value);
    void set_forward_soft_limit(double value);
    void set_forward_soft_limit_enable(bool enabled);
    void set_reverse_soft_limit(double value);
    void set_reverse_soft_limit_enable(bool enabled);
    void set_feedback_sensor_coefficient(double value);
    void set_voltage_compensation_saturation(double value);
    void set_voltage_compensation_enabled(bool enabled);
    void set_inverted(bool enabled);
    void set_sensor_phase_inverted(bool enabled);
    void set_neutral_mode(NeutralMode mode);
    void set_open_loop_ramp(double value);
    void set_closed_loop_ramp(double value);
    void set_supply_current_limit(bool enabled, double current_limit, double trigger_current, double trigger_time);
    void set_stator_current_limit(bool enabled, double current_limit, double trigger_current, double trigger_time);
    void set_follower(bool enabled, uint8_t master_id);
    void set_forward_limit_switch(LimitSwitchSource forward_limit_switch_source, LimitSwitchNormal forward_limit_switch_normal);
    void set_reverse_limit_switch(LimitSwitchSource reverse_limit_switch_source, LimitSwitchNormal reverse_limit_switch_normal);
    void set_peak_output_forward(double value);
    void set_peak_output_reverse(double value);
    void set_defaults();

protected:
    MotorConfig() { };

friend class MotorMaster;
friend class Motor;
};

class Motor
{
public:

    enum class Motor_Type
    {
        TALON_FX=0,
        TALON_SRX=1,
    };

    enum class Control_Mode : int
    {
        PERCENT_OUTPUT=0,
        POSITION=1,
        VELOCITY=2,
        CURRENT=3,
        MOTION_PROFILE=6,
        MOTION_MAGIC=7,
        MOTION_PROFILE_ARC=10,
        MUSIC_TONE=13,
        DISABLED=15,
    };

    Motor(uint8_t id, Motor_Type type);
    void set(Control_Mode mode, double output, double arbitrary_feedforward);
    MotorConfig& config();

private:
    Motor() = delete;
    uint8_t id;

    std::recursive_mutex mValueLock;
    Control_Mode mControlMode {Control_Mode::PERCENT_OUTPUT};
    double mOutput = 0;
    double mArbFF = 0;

friend class MotorMaster;
};
#endif