#pragma once

#include <stdint.h>

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

    Solenoid(uint8_t id, SolenoidType type);
    void set(SolenoidState state);
private:
    uint8_t id;
    SolenoidType type = SolenoidType::SINGLE;
    Solenoid() = delete;
    friend class SolenoidManager;
};