#include <cstdint>
#include "rio_control_node/Motor_Control.h"
#include "rio_control_node/Motor_Configuration.h"
#include "ck_utilities/Motor.hpp"

#include "ros/ros.h"
#include <mutex>
#include <map>
#include <thread>

extern ros::NodeHandle* node;

static std::recursive_mutex motor_mutex;

class MotorMaster
{
public:

    static void create_motor_config(uint8_t id, Motor::Motor_Type type)
    {
        std::lock_guard<std::recursive_mutex> lock(motor_mutex);
        if (configuration_map.find(id) != configuration_map.end())
        {
            return;
        }
        configuration_map[id] = new MotorConfig();
        configuration_map[id]->motor_id = id;
        configuration_map[id]->active_config.motor_id = id;
        configuration_map[id]->pending_config.motor_id = id;
        configuration_map[id]->active_config.motor_config.id = id;
        configuration_map[id]->pending_config.motor_config.id = id;
        configuration_map[id]->active_config.motor_config.controller_type = (uint8_t) type;
        configuration_map[id]->pending_config.motor_config.controller_type = (uint8_t) type;
        configuration_map[id]->set_defaults();
        configuration_map[id]->apply();
    }

    static MotorConfig * retrieve_configuration(uint8_t id)
    {
        std::lock_guard<std::recursive_mutex> lock(motor_mutex);
        if(configuration_map.find(id) == configuration_map.end())
        {
            return nullptr;
        }
        return configuration_map[id];
    }

    MotorMaster()
    {
        std::lock_guard<std::recursive_mutex> lock(motor_mutex);
        control_publisher = node->advertise<rio_control_node::Motor_Control>("/MotorControl", 50);
        config_publisher = node->advertise<rio_control_node::Motor_Configuration>("/MotorConfiguration", 50);

        motor_master_thread = new std::thread(motor_master_loop);
    }

    ~MotorMaster()
    {
        try
        {
            std::lock_guard<std::recursive_mutex> lock(motor_mutex);
            for(std::map<uint8_t, MotorConfig *>::iterator i = configuration_map.begin();
                i != configuration_map.end();
                i++)
            {
                delete (*i).second;
                (*i).second = nullptr;
            }
            configuration_map.clear();

            motor_master_thread->join();
        }
        catch ( ... ) { }

        try
        {
            motor_master_thread->join();
        }
        catch ( ... ) { }
    }

private:
    static std::thread * motor_master_thread;
    static ros::Publisher config_publisher;
    static ros::Publisher control_publisher;
    static std::map<uint8_t, MotorConfig *> configuration_map;

    static void send_motor_configs()
    {
        std::lock_guard<std::recursive_mutex> lock(motor_mutex);

        rio_control_node::Motor_Configuration config_list;

        for(std::map<uint8_t, MotorConfig *>::iterator i = configuration_map.begin();
            i != configuration_map.end();
            i++)
        {
            config_list.motors.push_back((*i).second->active_config.motor_config);
        }

        config_publisher.publish(config_list);
    }

    static void send_follower_controls()
    {
        std::lock_guard<std::recursive_mutex> lock(motor_mutex);

        rio_control_node::Motor_Control motor_control_list;

        for(std::map<uint8_t, MotorConfig *>::iterator i = configuration_map.begin();
            i != configuration_map.end();
            i++)
        {
            if((*i).second->active_config.motor_config.invert_type == rio_control_node::Motor_Config::FOLLOW_MASTER ||
               (*i).second->active_config.motor_config.invert_type == rio_control_node::Motor_Config::OPPOSE_MASTER)
            {
                rio_control_node::Motor motor;
                motor.controller_type = (*i).second->active_config.motor_config.controller_type;
                motor.id = (*i).second->active_config.motor_config.id;
                motor.control_mode = rio_control_node::Motor::FOLLOWER;
                motor.output_value = (*i).second->active_config.master_id;
                motor_control_list.motors.push_back(motor);
            }
        }

        control_publisher.publish(motor_control_list);
    }

    static void motor_master_loop()
    {
        ros::Rate timer(10);
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

std::map<uint8_t, MotorConfig *> MotorMaster::configuration_map;
std::thread * MotorMaster::motor_master_thread;
ros::Publisher MotorMaster::config_publisher;
ros::Publisher MotorMaster::control_publisher;

static MotorMaster * motor_master = nullptr;

void MotorConfig::apply()
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->active_config = this->pending_config;
}

void MotorConfig::set_fast_master(bool enable)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    if(enable)
    {
        this->pending_config.motor_config.controller_mode = rio_control_node::Motor_Config::FAST_MASTER;
    }
    else
    {
        this->pending_config.motor_config.controller_mode = rio_control_node::Motor_Config::MASTER;
    }
}

void MotorConfig::set_kP(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.kP = value;
}

void MotorConfig::set_kI(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.kI = value;
}

void MotorConfig::set_kD(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.kD = value;
}

void MotorConfig::set_kF(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.kF = value;
}

void MotorConfig::set_i_zone(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.iZone = value;
}

void MotorConfig::set_max_i_accum(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.max_i_accum = value;
}

void MotorConfig::set_allowed_closed_loop_error(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.allowed_closed_loop_error = value;
}

void MotorConfig::set_max_closed_loop_peak_output(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.max_closed_loop_peak_output = value;
}

void MotorConfig::set_motion_cruise_velocity(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.motion_cruise_velocity = value;
}

void MotorConfig::set_motion_acceleration(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.motion_acceleration = value;
}

void MotorConfig::set_motion_s_curve_strength(int32_t value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.motion_s_curve_strength = value;
}

void MotorConfig::set_forward_soft_limit(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.forward_soft_limit = value;
}

void MotorConfig::set_forward_soft_limit_enable(bool enabled)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.forward_soft_limit_enable = enabled;
}

void MotorConfig::set_reverse_soft_limit(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.reverse_soft_limit = value;
}

void MotorConfig::set_reverse_soft_limit_enable(bool enabled)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.reverse_soft_limit_enable = enabled;
}

void MotorConfig::set_feedback_sensor_coefficient(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.feedback_sensor_coefficient = value;
}

void MotorConfig::set_voltage_compensation_saturation(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.voltage_compensation_saturation = value;
}

void MotorConfig::set_voltage_compensation_enabled(bool enabled)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.voltage_compensation_enabled = enabled;
}

void MotorConfig::set_inverted(bool enabled)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
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
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.sensor_phase_inverted = enabled;
}

void MotorConfig::set_neutral_mode(MotorConfig::NeutralMode mode)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.neutral_mode = (uint8_t) mode;
}

void MotorConfig::set_open_loop_ramp(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.open_loop_ramp = value;
}

void MotorConfig::set_closed_loop_ramp(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.closed_loop_ramp = value;
}

void MotorConfig::set_supply_current_limit(bool enabled, double current_limit, double trigger_current, double trigger_time)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.supply_current_limit_config.enable = enabled;
    this->pending_config.motor_config.supply_current_limit_config.current_limit = current_limit;
    this->pending_config.motor_config.supply_current_limit_config.trigger_threshold_current = trigger_current;
    this->pending_config.motor_config.supply_current_limit_config.trigger_threshold_time = trigger_time;
}

void MotorConfig::set_stator_current_limit(bool enabled, double current_limit, double trigger_current, double trigger_time)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.stator_current_limit_config.enable = enabled;
    this->pending_config.motor_config.stator_current_limit_config.current_limit = current_limit;
    this->pending_config.motor_config.stator_current_limit_config.trigger_threshold_current = trigger_current;
    this->pending_config.motor_config.stator_current_limit_config.trigger_threshold_time = trigger_time;
}

void MotorConfig::set_follower(bool enabled, uint8_t master_id)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
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
        this->pending_config.motor_config.controller_mode = rio_control_node::Motor_Config::FOLLOWER;
        return;
    }
    this->pending_config.master_id = 0;
    this->pending_config.motor_config.controller_mode = rio_control_node::Motor_Config::MASTER;
    if(this->pending_config.motor_config.invert_type == rio_control_node::Motor_Config::OPPOSE_MASTER ||
       this->pending_config.motor_config.invert_type == rio_control_node::Motor_Config::INVERT_MOTOR_OUTPUT)
    {
        this->pending_config.motor_config.invert_type = rio_control_node::Motor_Config::INVERT_MOTOR_OUTPUT;
        return;
    }
    this->pending_config.motor_config.invert_type = rio_control_node::Motor_Config::NONE;
}

void MotorConfig::set_forward_limit_switch(LimitSwitchSource forward_limit_switch_source, LimitSwitchNormal forward_limit_switch_normal)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.forward_limit_switch_source = (int8_t)forward_limit_switch_source;
    this->pending_config.motor_config.forward_limit_switch_normal = (int8_t)forward_limit_switch_normal;
}

void MotorConfig::set_reverse_limit_switch(LimitSwitchSource reverse_limit_switch_source, LimitSwitchNormal reverse_limit_switch_normal)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.reverse_limit_switch_source = (int8_t)reverse_limit_switch_source;
    this->pending_config.motor_config.reverse_limit_switch_normal = (int8_t)reverse_limit_switch_normal;
}

void MotorConfig::set_defaults()
{    
    this->set_fast_master(false);
    this->set_kP(0.0);
    this->set_kI(0.0);
    this->set_kD(0.0);
    this->set_kF(0.0);
    this->set_i_zone(0.0);
    this->set_max_i_accum(0.0);
    this->set_allowed_closed_loop_error(0.0);
    this->set_max_closed_loop_peak_output(0.0);
    this->set_motion_cruise_velocity(0.0);
    this->set_motion_acceleration(0.0);
    this->set_motion_s_curve_strength(0);
    this->set_forward_soft_limit(0.0);
    this->set_forward_soft_limit_enable(false);
    this->set_reverse_soft_limit(0.0);
    this->set_reverse_soft_limit_enable(false);
    this->set_feedback_sensor_coefficient(0.0);
    this->set_voltage_compensation_saturation(0.0);
    this->set_inverted(false);
    this->set_sensor_phase_inverted(false);
    this->set_neutral_mode(MotorConfig::NeutralMode::COAST);
    this->set_open_loop_ramp(0.0);
    this->set_closed_loop_ramp(0.0);
    this->set_supply_current_limit(false, 0.0, 0.0, 0.0);
    this->set_stator_current_limit(false, 0.0, 0.0, 0.0);
    this->set_follower(false, 0);
    this->set_forward_limit_switch(LimitSwitchSource::FeedbackConnector, LimitSwitchNormal::NormallyOpen);
    this->set_reverse_limit_switch(LimitSwitchSource::FeedbackConnector, LimitSwitchNormal::NormallyOpen);
}

Motor::Motor(uint8_t id, Motor_Type type)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    if(motor_master == nullptr)
    {
        motor_master = new MotorMaster();
    }
    this->id = id;
    motor_master->create_motor_config(id, type);
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
    return *(motor_master->retrieve_configuration(this->id));
}
