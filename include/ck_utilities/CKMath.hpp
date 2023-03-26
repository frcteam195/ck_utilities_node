#pragma once

#include "CKMathConstants.hpp"
#include "Units.hpp"
#include <cmath>
#include <map>
#include <iostream>

//Repetitive macro power to improve timing critical power speed https://stackoverflow.com/a/8556436
//Provide macro expansion of pow up to power of 10
#define REP1(X) X
#define REP2(X) REP1(X) * X
#define REP3(X) REP2(X) * X
#define REP4(X) REP3(X) * X
#define REP5(X) REP4(X) * X
#define REP6(X) REP5(X) * X
#define REP7(X) REP6(X) * X
#define REP8(X) REP7(X) * X
#define REP9(X) REP8(X) * X
#define REP10(X) REP9(X) * X

#define CKPOW(X, ONES) REP##ONES(X)

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
        inline T normalizeWithDeadband(T val, T deadband) {
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
        inline T radians_per_second_to_ticks_per_100ms(T rad_s, T rotations_per_tick_vel)
        {
            return rad_s / (PI * 2.0) / rotations_per_tick_vel / 10.0;
        }

        // wrap x -> [0,max)
        template <typename T>
        inline T wrapMax(T x, T max)
        {
            /* integer math: (max + x % max) % max */
            return std::fmod(max + std::fmod(x, max), max);
        }

        // wrap x -> [min,max)
        template <typename T>
        inline T wrapMinMax(T x, T min, T max)
        {
            return min + wrapMax(x - min, max - min);
        }

        template <typename T>
        inline T normalize_to_2_pi(T value)
        {
            return wrapMinMax<T>(value, 0, (2.0 * M_PI));
        }

        template <typename T>
        inline T normalize_to_minus_pi_to_pi(T value)
        {
            return wrapMinMax<T>(value, -M_PI, M_PI);
        }

        template <typename T>
        inline T hypotenuse(T x, T y)
        {
            return std::sqrt(CKPOW(x, 2) + CKPOW(y,2));
        }

        template <typename T>
        inline T hypotenuse(T x, T y, T z)
        {
            return std::sqrt(CKPOW(x, 2) + CKPOW(y,2) + CKPOW(z,2));
        }

        template <typename T>
        inline T polar_angle_rad(T x, T y)
        {
            return normalize_to_2_pi(std::atan2(y, x));
        }

        template <typename T>
        inline T map(T value, T low1, T high1, T low2, T high2)
        {
            return low2 + (value - low1) * (high2 - low2) / (high1 - low1);
        }
    } // namespace math
} // namespace ck