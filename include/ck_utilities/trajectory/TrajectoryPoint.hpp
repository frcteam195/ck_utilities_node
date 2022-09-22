#pragma once

#include "ck_utilities/team254_geometry/State.hpp"

#include <type_traits>

namespace ck
{
    namespace trajectory
    {
        template <class S>
        struct TrajectoryPoint
        {
            static_assert(std::is_base_of<ck::team254_geometry::State<S>, S>::value, "S must inherit from State<S>");
            S state_;
            int index_;
        };
    }
}
