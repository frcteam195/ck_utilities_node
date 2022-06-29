#pragma once

#include "ck_utilities/geometry/State.hpp"

namespace ck
{
    namespace geometry
    {
        template <class S>
        class ICurvature : public State<S>
        {
        public:
            virtual double getCurvature() const = 0;
            virtual double getDCurvatureDs() const = 0;
        };
    } // namespace geometry
} // namespace ck