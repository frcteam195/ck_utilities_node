#pragma once

#include "ck_utilities/team254_geometry/State.hpp"
#include "TrajectoryPoint.hpp"

#include <type_traits>

namespace ck
{
    namespace trajectory
    {
        template <class S, class T>
        class TrajectorySamplePoint
        {
        public:
            static_assert(std::is_base_of<ck::team254_geometry::State<S>, S>::value, "S must inherit from State<S>");
            static_assert(std::is_base_of<ck::team254_geometry::State<T>, T>::value, "T must inherit from State<T>");
            S state_;
            T heading_;
            int index_floor_;
            int index_ceil_;


            int index_floor(){ return index_floor_; }
            int index_ceil(){ return index_ceil_; }
            S state(){ return state_; }
            T heading(){ return heading_; }

            TrajectorySamplePoint()
                : state_(),
                  heading_(),
                  index_floor_(0),
                  index_ceil_(0)
            {
            }

            TrajectorySamplePoint(const TrajectoryPoint<S, T> &point)
                : state_(point.state_),
                  heading_(point.heading_),
                  index_floor_(point.index_),
                  index_ceil_(point.index_)
            {
            }

            TrajectorySamplePoint(S state, T heading, int index_floor, int index_ceil)
                : state_(state),
                  heading_(heading),
                  index_floor_(index_floor),
                  index_ceil_(index_ceil)
            {
            }

        };
    } // namespace trajectory
} // namespace ck
