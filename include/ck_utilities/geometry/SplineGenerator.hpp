#pragma once

#include "ck_utilities/CKMath.hpp"
#include "ck_utilities/geometry/Geometry.hpp"
#include "ck_utilities/geometry/QuinticHermiteSpline.hpp"

#include <cmath>
#include <vector>

namespace ck
{
    namespace geometry
    {
        class SplineGenerator
        {
        private:
            static constexpr double kMaxDX = 2.0;     //inches
            static constexpr double kMaxDY = 0.05;    //inches
            static constexpr double kMaxDTheta = 0.1; //radians!
            static constexpr double kMinSampleSize = 1;

            static void getSegmentArc(QuinticHermiteSpline &s, std::vector<ck::geometry::Pose2dWithCurvature> &rv, double t0, double t1, double maxDx, double maxDy, double maxDTheta);

        public:
            /**
             * Converts a spline into a list of Twist2d's.
             *
             * @param s  the spline to parametrize
             * @param t0 starting percentage of spline to parametrize
             * @param t1 ending percentage of spline to parametrize
             * @return list of Pose2dWithCurvature that approximates the original spline
             */
            static std::vector<ck::geometry::Pose2dWithCurvature> parameterizeSpline(QuinticHermiteSpline &s, double maxDx, double maxDy, double maxDTheta, double t0, double t1);

            /**
             * Convenience function to parametrize a spline from t 0 to 1
             */
            static std::vector<ck::geometry::Pose2dWithCurvature> parameterizeSpline(QuinticHermiteSpline &s);
            static std::vector<ck::geometry::Pose2dWithCurvature> parameterizeSpline(QuinticHermiteSpline &s, double maxDx, double maxDy, double maxDTheta);
            static std::vector<ck::geometry::Pose2dWithCurvature> parameterizeSplines(std::vector<QuinticHermiteSpline> &splines);
            static std::vector<ck::geometry::Pose2dWithCurvature> parameterizeSplines(std::vector<QuinticHermiteSpline> &splines, double maxDx, double maxDy, double maxDTheta);
        };
    } // namespace geometry
} // namespace ck