#pragma once

#include "ck_utilities/team254_geometry/State.hpp"
#include "TrajectorySamplePoint.hpp"

#include <type_traits>

namespace ck
{
    namespace trajectory
    {
        template <class S, class T>
        class Trajectory;

        template <class S, class T>
        class TrajectoryView
        {
            static_assert(std::is_base_of<ck::team254_geometry::State<S>, S>::value, "S must inherit from State");
            static_assert(std::is_base_of<ck::team254_geometry::State<T>, T>::value, "T must inherit from State");

        public:
            virtual ~TrajectoryView(){}
            virtual TrajectorySamplePoint<S, T> sample(double interpolant) = 0;
            virtual double first_interpolant() const = 0;
            virtual double last_interpolant() const = 0;
            virtual Trajectory<S, T> trajectory() = 0;
        };
    } // namespace trajectory
} // namespace ck
