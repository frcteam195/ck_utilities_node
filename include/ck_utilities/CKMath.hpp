#pragma once

#include "CKMathConstants.hpp"
#include "Units.hpp"
#include <cmath>
#include <map>

#ifdef ROSCPP_ROS_H
#include "CKMath_ros.hpp"
#endif

namespace ck
{
    namespace math
    {
        template <typename T>
        inline T max(T a, T b)
        {
            return a > b ? a : b;
        }

        template <typename T>
        inline T min(T a, T b)
        {
            return a < b ? a : b;
        }

        template <typename T>
        inline int signum(T val)
        {
            return (T(0) < val) - (val < T(0));
        }

        template <typename T>
        inline bool inRange(T val, T delta)
        {
            return val >= -delta && val <= delta;
        }

        template <typename T>
        inline bool inRange(T val, T minVal, T maxVal)
        {
            return val >= minVal && val <= maxVal;
        }

        template <typename T>
        inline bool epsilonEquals(T const &a, T const &b, T epsilon)
        {
            return (a - epsilon <= b) && (a + epsilon >= b);
        }

        template <typename T>
        inline bool epsilonEquals(T const &a, T const &b)
        {
            return epsilonEquals(a, b, kEpsilon);
        }

        template <typename T>
        inline T limit(T v, T minVal, T maxVal)
        {
            return min(maxVal, max(minVal, v));
        }

        template <typename T>
        inline T limit(T v, T maxMagnitude)
        {
            return limit(v, -maxMagnitude, maxMagnitude);
        }

        template <typename T>
        inline T interpolate(T a, T b, T x)
        {
            x = limit(x, 0.0, 1.0);
            return a + (b - a) * x;
        }

        template <typename T>
        inline T handleDeadband(T val, T deadband) {
            return (std::fabs(val) > std::fabs(deadband)) ? val : 0.0;
        };

        template <typename T>
        T normalizeWithDeadband(T val, T deadband) {
            val = handleDeadband(val, deadband);

            if (val != 0)
            {
                val = signum(val) * ((std::fabs(val) - deadband) / (1.0 - deadband));
            }

            return val;
        }

        template <typename K, typename V>
        inline V interpolate(const std::map<K, V> &data, K x)
        {
            typedef typename std::map<K, V>::const_iterator i_t;

            i_t i = data.upper_bound(x);
            if (i == data.end())
            {
                return (--i)->second;
            }
            if (i == data.begin())
            {
                return i->second;
            }
            i_t l = i;
            --l;

            const K delta = (x - l->first) / (i->first - l->first);
            return delta * i->second + (1 - delta) * l->second;
        }

        template <typename K, typename V>
        inline V interpolateGeometry2d(const std::map<K, V> &data, K x)
        {
            typedef typename std::map<K, V>::const_iterator i_t;

            i_t i = data.upper_bound(x);
            if (i == data.end())
            {
                return (--i)->second;
            }
            if (i == data.begin())
            {
                return i->second;
            }
            i_t l = i;
            --l;

            const K delta = (x - l->first) / (i->first - l->first);
            return i->second.interpolate(l->second, delta);
        }

        template <typename T>
        T radians_per_second_to_ticks_per_100ms(T rad_s, T rotations_per_tick_vel)
        {
            return rad_s / (PI * 2.0) / rotations_per_tick_vel / 10.0;
        }

    } // namespace math
} // namespace ck