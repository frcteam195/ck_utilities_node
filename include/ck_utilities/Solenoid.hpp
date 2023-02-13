#pragma once
#if __has_include("ros/ros.h")
#include <stdint.h>

class SolenoidMaster;

class Solenoid
{
public:
    enum class SolenoidType
    {
        SINGLE = 0,
        DOUBLE = 1,
    };

    enum class SolenoidState
    {
        OFF = 0,
        ON = 1,
        FORWARD = 1,
        REVERSE = 2,
    };

    Solenoid(uint32_t id, uint32_t module_id, SolenoidType type);
    void set(SolenoidState state);
private:
    uint32_t id;
    SolenoidType type = SolenoidType::SINGLE;
    SolenoidState mOutput = SolenoidState::OFF;
    Solenoid() = delete;
friend class SolenoidMaster;
};
#endif