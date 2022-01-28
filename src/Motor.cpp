#include <cstdint>
#include "rio_control_node/Motor_Control.h"
#include "rio_control_node/Motor_Configuration.h"
#include "ck_utilities/Motor.hpp"

#include "ros/ros.h"
#include <mutex>
#include <map>
#include <thread>

extern ros::NodeHandle* node;

static std::mutex motor_mutex;

class MotorMaster
{
public:

    static void register_motor_data(MotorData motor_data)
    {
        std::lock_guard<std::mutex> lock(motor_mutex);
        registered_motors[motor_data.motor_id] = motor_data;
    }
    MotorMaster()
    {
        std::lock_guard<std::mutex> lock(motor_mutex);
        control_publisher = node->advertise<rio_control_node::Motor_Control>("MotorControl", 50);
        config_publisher = node->advertise<rio_control_node::Motor_Configuration>("MotorConfiguration", 50);

        motor_master_thread = new std::thread(motor_master_loop);
    }

    ~MotorMaster()
    {
        try
        {
            motor_master_thread->join();
        }
        catch ( ... ) { }
    }

private:
    static std::map<int, MotorData> registered_motors;
    static std::thread * motor_master_thread;
    static ros::Publisher config_publisher;
    static ros::Publisher control_publisher;

    static void send_motor_configs()
    {
        std::lock_guard<std::mutex> lock(motor_mutex);

        rio_control_node::Motor_Configuration config_list;

        for(std::map<int, MotorData>::iterator i = registered_motors.begin();
            i != registered_motors.end();
            i++)
        {
            config_list.motors.push_back((*i).second.motor_config);
        }

        config_publisher.publish(config_list);
    }

    static void send_follower_controls()
    {
        std::lock_guard<std::mutex> lock(motor_mutex);

        rio_control_node::Motor_Control motor_control_list;

        for(std::map<int, MotorData>::iterator i = registered_motors.begin();
            i != registered_motors.end();
            i++)
        {
            if((*i).second.motor_config.controller_type == rio_control_node::Motor_Config::FOLLOW_MASTER ||
               (*i).second.motor_config.controller_type == rio_control_node::Motor_Config::OPPOSE_MASTER)
            {
                rio_control_node::Motor motor;
                motor.controller_type = (*i).second.motor_config.controller_type;
                motor.id = (*i).second.motor_config.id;
                motor.control_mode = rio_control_node::Motor::FOLLOWER;
                motor.output_value = (*i).second.master_id;
                motor_control_list.motors.push_back(motor);
            }
        }

        control_publisher.publish(motor_control_list);
    }

    static void motor_master_loop()
    {
        ros::Rate timer(100);
        while(ros::ok())
        {
            send_motor_configs();
            send_follower_controls();
            timer.sleep();
        }
    }

friend class Motor;
friend class MotorConfig;
};

std::map<int, MotorData> MotorMaster::registered_motors;
std::thread * MotorMaster::motor_master_thread;
ros::Publisher MotorMaster::config_publisher;
ros::Publisher MotorMaster::control_publisher;

static MotorMaster motor_master;

void MotorConfig::apply()
{
    this->active_config = this->pending_config;
    motor_master.register_motor_data(this->active_config);
}

void MotorConfig::set_fast_master(bool enable)
{
    if(enable)
    {
        this->pending_config.motor_config.controller_mode = rio_control_node::Motor_Config::FAST_MASTER;
    }
    else
    {
        this->pending_config.motor_config.controller_mode = rio_control_node::Motor_Config::FAST_MASTER;
    }
}

void MotorConfig::set_kP(double value)
{
    this->pending_config.motor_config.kP = value;
}

void MotorConfig::set_kI(double value)
{
    this->pending_config.motor_config.kI = value;
}

void MotorConfig::set_kD(double value)
{
    this->pending_config.motor_config.kD = value;
}

void MotorConfig::set_kF(double value)
{
    this->pending_config.motor_config.kF = value;
}

void MotorConfig::set_i_zone(double value)
{
    this->pending_config.motor_config.iZone = value;
}

void MotorConfig::set_max_i_accum(double value)
{
    this->pending_config.motor_config.max_i_accum = value;
}

void MotorConfig::set_allowed_closed_loop_error(double value)
{
    this->pending_config.motor_config.allowed_closed_loop_error = value;
}

void MotorConfig::set_max_closed_loop_peak_output(double value)
{
    this->pending_config.motor_config.max_closed_loop_peak_output = value;
}

void MotorConfig::set_motion_cruise_velocity(double value)
{
    this->pending_config.motor_config.motion_cruise_velocity = value;
}

void MotorConfig::set_motion_acceleration(double value)
{
    this->pending_config.motor_config.motion_acceleration = value;
}

void MotorConfig::set_motion_s_curve_strength(int32_t value)
{
    this->pending_config.motor_config.motion_s_curve_strength = value;
}

void MotorConfig::set_forward_soft_limit(double value)
{
    this->pending_config.motor_config.forward_soft_limit = value;
}

void MotorConfig::set_forward_soft_limit_enable(bool enabled)
{
    this->pending_config.motor_config.forward_soft_limit_enable = enabled;
}

void MotorConfig::set_reverse_soft_limit(double value)
{
    this->pending_config.motor_config.reverse_soft_limit = value;
}

void MotorConfig::set_reverse_soft_limit_enable(bool enabled)
{
    this->pending_config.motor_config.reverse_soft_limit_enable = enabled;
}

void MotorConfig::set_feedback_sensor_coefficient(double value)
{
    this->pending_config.motor_config.feedback_sensor_coefficient = value;
}

void MotorConfig::set_voltage_compensation_saturation(double value)
{
    this->pending_config.motor_config.voltage_compensation_saturation = value;
}

void MotorConfig::set_inverted(bool enabled)
{
    if(this->pending_config.motor_config.invert_type == rio_control_node::Motor_Config::FOLLOW_MASTER ||
       this->pending_config.motor_config.invert_type == rio_control_node::Motor_Config::OPPOSE_MASTER)
    {
        this->pending_config.motor_config.invert_type = enabled ? rio_control_node::Motor_Config::OPPOSE_MASTER : rio_control_node::Motor_Config::FOLLOW_MASTER;
        return;
    }
    this->pending_config.motor_config.invert_type = enabled ? rio_control_node::Motor_Config::INVERT_MOTOR_OUTPUT : rio_control_node::Motor_Config::NONE;

}

void MotorConfig::set_sensor_phase_inverted(bool enabled)
{
    this->pending_config.motor_config.sensor_phase_inverted = enabled;
}

void MotorConfig::set_neutral_mode(MotorConfig::NeutralMode mode)
{
    this->pending_config.motor_config.neutral_mode = (uint8_t) mode;
}

void MotorConfig::set_open_loop_ramp(double value)
{
    this->pending_config.motor_config.open_loop_ramp = value;
}

void MotorConfig::set_closed_loop_ramp(double value)
{
    this->pending_config.motor_config.closed_loop_ramp = value;
}

void MotorConfig::set_supply_current_limit(bool enabled, double current_limit, double trigger_current, double trigger_time)
{
    this->pending_config.motor_config.supply_current_limit_config.enable = enabled;
    this->pending_config.motor_config.supply_current_limit_config.current_limit = current_limit;
    this->pending_config.motor_config.supply_current_limit_config.trigger_threshold_current = trigger_current;
    this->pending_config.motor_config.supply_current_limit_config.trigger_threshold_time = trigger_time;
}

void MotorConfig::set_stator_current_limit(bool enabled, double current_limit, double trigger_current, double trigger_time)
{
    this->pending_config.motor_config.stator_current_limit_config.enable = enabled;
    this->pending_config.motor_config.stator_current_limit_config.current_limit = current_limit;
    this->pending_config.motor_config.stator_current_limit_config.trigger_threshold_current = trigger_current;
    this->pending_config.motor_config.stator_current_limit_config.trigger_threshold_time = trigger_time;
}

void MotorConfig::set_follower(bool enabled, uint8_t master_id)
{
    if(enabled)
    {
        this->pending_config.master_id = master_id;
        if(this->pending_config.motor_config.invert_type == rio_control_node::Motor_Config::OPPOSE_MASTER ||
        this->pending_config.motor_config.invert_type == rio_control_node::Motor_Config::INVERT_MOTOR_OUTPUT)
        {
            this->pending_config.motor_config.invert_type = rio_control_node::Motor_Config::OPPOSE_MASTER;
            return;
        }
        this->pending_config.motor_config.invert_type = rio_control_node::Motor_Config::FOLLOW_MASTER;
        return;
    }
    this->pending_config.master_id = 0;
    if(this->pending_config.motor_config.invert_type == rio_control_node::Motor_Config::OPPOSE_MASTER ||
       this->pending_config.motor_config.invert_type == rio_control_node::Motor_Config::INVERT_MOTOR_OUTPUT)
    {
        this->pending_config.motor_config.invert_type = rio_control_node::Motor_Config::INVERT_MOTOR_OUTPUT;
        return;
    }
    this->pending_config.motor_config.invert_type = rio_control_node::Motor_Config::NONE;
}

void MotorConfig::set_defaults()
{

}

Motor::Motor(uint8_t id, Motor_Type type)
{
    this->config().pending_config.motor_config.controller_type = (uint8_t) type;
    this->config().pending_config.motor_config.id = id;
    this->config().set_defaults();
}

void Motor::set(Control_Mode mode, double output, double arbitrary_feedforward)
{
    rio_control_node::Motor_Control motors;
    rio_control_node::Motor motor;
    motor.controller_type = this->config().active_config.motor_config.controller_type;
    motor.id = this->config().active_config.motor_id;
    motor.control_mode = (uint8_t) mode;
    motor.output_value = output;
    motor.arbitrary_feedforward = arbitrary_feedforward;
    motors.motors.push_back(motor);
    static ros::Publisher motor_control_pub = node->advertise<rio_control_node::Motor_Control>("MotorControl", 50);
    motor_control_pub.publish(motors);
}

MotorConfig& Motor::config()
{
    return this->config_structure;
}
