#pragma once

#include <stdint.h>

class Piston
{
public:
    enum class PistonType
    {
        SINGLE = 0,
        DOUBLE = 1,
    };

    enum class PistonState
    {
        OFF = 0,
        ON = 1,
        FORWARD = 1,
        REVERSE = 2,
    };

    Piston(uint8_t id, PistonType type);
    void set(PistonState state);
private:
    uint8_t id;
    PistonType type = PistonType::SINGLE;
    Piston() = delete;
    friend class PistonManager;
};