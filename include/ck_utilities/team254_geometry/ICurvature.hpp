#pragma once

#include "ck_utilities/team254_geometry/State.hpp"

namespace ck
{
    namespace team254_geometry
    {
        template <class S>
        class ICurvature : public State<S>
        {
        public:
            virtual double getCurvature() const = 0;
            virtual double getDCurvatureDs() const = 0;
        };
    } // namespace team254_geometry
} // namespace ck