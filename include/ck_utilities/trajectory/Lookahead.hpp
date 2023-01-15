#pragma once

#include "ck_utilities/CKMath.hpp"

namespace ck
{
    namespace trajectory
    {
        class Lookahead
        {

        public:
            Lookahead(double min_dis, double max_dis, double min_spd, double max_spd)
                : min_distance(min_dis),
                  max_distance(max_dis),
                  min_speed(min_spd),
                  max_speed(max_spd),
                  delta_distance(max_distance - min_distance),
                  delta_speed(max_speed - min_speed) {}

            double getLookaheadForSpeed(double speed)
            {
                double lookahead = delta_distance * (speed - min_speed) / delta_speed + min_distance;
                return std::isnan(lookahead) ? min_distance : math::max(min_distance, math::min(max_distance, lookahead));
            }

            const double min_distance;
            const double max_distance;
            const double min_speed;
            const double max_speed;

        protected:
            const double delta_distance;
            const double delta_speed;
        };
    } // namespace trajectory
    
} // namespace ck
