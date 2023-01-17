#pragma once

#include "ck_utilities/team254_geometry/State.hpp"

namespace ck
{
    namespace team254_geometry
    {
        class Rotation2d;

        template <class S>
        class IRotation2d : public State<S>
        {
        public:
            virtual Rotation2d getRotation() const = 0;
            virtual S rotateBy(const Rotation2d &other) const = 0;
            virtual S mirror() const = 0;
        };
    } // namespace team254_geometry
} // namespace ck