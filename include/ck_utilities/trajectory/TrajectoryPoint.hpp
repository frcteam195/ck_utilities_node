#pragma once

#include "ck_utilities/team254_geometry/State.hpp"

#include <type_traits>

namespace ck
{
    namespace trajectory
    {
        template <class S, class T>
        struct TrajectoryPoint
        {
            static_assert(std::is_base_of<ck::team254_geometry::State<S>, S>::value, "S must inherit from State<S>");
            static_assert(std::is_base_of<ck::team254_geometry::State<T>, T>::value, "T must inherit from State<T>");
            S state_;
            T heading_;
            int index_;
        };
    }
}
