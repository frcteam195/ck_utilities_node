#pragma once

#include "ck_utilities/CKMath.hpp"

#include <cmath>
#include <ostream>
#include <iomanip>

namespace ck
{
    namespace geometry
    {
        class Twist2d
        {
        public:
            double dx;
            double dy;
            double dtheta; // Radians!

            static const Twist2d &identity();
            Twist2d();
            Twist2d(double dx, double dy, double dtheta);

            friend std::ostream &operator<<(std::ostream &os, const Twist2d &tw2d);

            Twist2d scaled(double scale) const;
            double norm() const;
            double curvature() const;

        private:
        };
    } // namespace geometry
} // namespace ck