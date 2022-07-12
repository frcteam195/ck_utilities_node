#include "ck_utilities/Solenoid.hpp"
#if __has_include("ros/ros.h")
#include <thread>
#include <map>
#include <mutex>
#include "ros/ros.h"
#include "rio_control_node/Solenoid_Control.h"

static std::map<uint8_t, Solenoid *> solenoid_map;
static std::recursive_mutex solenoid_mutex;
static ros::Publisher control_publisher;

extern ros::NodeHandle * node;

class SolenoidMaster
{
public:
    static void store_solenoid_pointer(uint8_t id, Solenoid* solenoid)
    {
        std::lock_guard<std::recursive_mutex> lock(solenoid_mutex);
        solenoid_map[id] = solenoid;
    }

    static Solenoid * retrieve_solenoid(uint8_t id)
    {
        std::lock_guard<std::recursive_mutex> lock(solenoid_mutex);
        if(solenoid_map.find(id) == solenoid_map.end())
        {
            return nullptr;
        }
        return solenoid_map[id];
    }

    SolenoidMaster()
    {
        std::lock_guard<std::recursive_mutex> lock(solenoid_mutex);
        control_publisher = node->advertise<rio_control_node::Solenoid_Control>("/SolenoidControl", 50);
        solenoid_master_thread = new std::thread(solenoid_master_loop);
    }

    ~SolenoidMaster()
    {
        try
        {
            std::lock_guard<std::recursive_mutex> lock(solenoid_mutex);
            for(std::map<uint8_t, Solenoid *>::iterator i = solenoid_map.begin();
                i != solenoid_map.end();
                i++)
            {
                delete (*i).second;
                (*i).second = nullptr;
            }
            solenoid_map.clear();

            solenoid_master_thread->join();
        }
        catch ( ... ) { }

        try
        {
            solenoid_master_thread->join();
        }
        catch ( ... ) { }
    }

private:
    static std::thread * solenoid_master_thread;

    static void send_master_controls_periodic()
    {
        std::lock_guard<std::recursive_mutex> lock(solenoid_mutex);
        static rio_control_node::Solenoid_Control solenoid_control_list;
        solenoid_control_list.solenoids.clear();

        for(std::map<uint8_t, Solenoid *>::iterator i = solenoid_map.begin();
            i != solenoid_map.end();
            i++)
        {
            Solenoid* s = (*i).second;
            rio_control_node::Solenoid solenoid;
            solenoid.solenoid_type = (int8_t)s->type;
            solenoid.id = s->id;
            Solenoid::SolenoidState tmpState = s->mOutput;
            solenoid.output_value = (int8_t)tmpState;

            solenoid_control_list.solenoids.push_back(solenoid);
        }

        control_publisher.publish(solenoid_control_list);
    }

    static void solenoid_master_loop()
    {
        ros::Rate timer(10);
        while(ros::ok())
        {
            send_master_controls_periodic();
            timer.sleep();
        }
    }

friend class Solenoid;
};

static SolenoidMaster * solenoid_master = nullptr;
std::thread * SolenoidMaster::solenoid_master_thread;

Solenoid::Solenoid(uint8_t id, Solenoid::SolenoidType type)
{
    std::lock_guard<std::recursive_mutex> lock(solenoid_mutex);
    this->id = id;
    this->type = type;
    if(solenoid_master == nullptr)
    {
        solenoid_master = new SolenoidMaster();
    }
    this->id = id;
    solenoid_master->store_solenoid_pointer(id, this);
}

void Solenoid::set(Solenoid::SolenoidState state)
{
    std::lock_guard<std::recursive_mutex> lock(solenoid_mutex);
    mOutput = state;

    rio_control_node::Solenoid_Control solenoid_control;
    rio_control_node::Solenoid solenoid;
    solenoid.id = (int32_t) this->id;
    solenoid.solenoid_type = (int8_t) this->type;
    solenoid.module_type = rio_control_node::Solenoid::CTREPCM;
    solenoid.output_value = (int8_t) state;
    solenoid_control.solenoids.push_back(solenoid);

    control_publisher.publish(solenoid_control);
}
#endif