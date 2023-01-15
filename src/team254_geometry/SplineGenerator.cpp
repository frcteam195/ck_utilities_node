#include "ck_utilities/team254_geometry/SplineGenerator.hpp"

namespace ck
{
    namespace team254_geometry
    {
        void SplineGenerator::getSegmentArc(QuinticHermiteSpline &s,
                                            std::vector<Rotation2d> &headings,
                                            std::vector<trajectory::TrajectoryPoint<Pose2dWithCurvature, Rotation2d>> &rv,
                                            double t0,
                                            double t1,
                                            double maxDx,
                                            double maxDy,
                                            double maxDTheta,
                                            double totalTime)
        {
            Translation2d p0 = s.getPoint(t0);
            Translation2d p1 = s.getPoint(t1);
            Rotation2d r0 = s.getHeading(t0);
            Rotation2d r1 = s.getHeading(t1);
            Pose2d transformation(Translation2d(p0, p1).rotateBy(r0.inverse()), r1.rotateBy(r0.inverse()));
            Twist2d twist = Pose2d::log(transformation);
            if (twist.dy > maxDy || twist.dx > maxDx || twist.dtheta > maxDTheta)
            {
                getSegmentArc(s, headings, rv, t0, (t0 + t1) / 2, maxDx, maxDy, maxDTheta, totalTime);
                getSegmentArc(s, headings, rv, (t0 + t1) / 2, t1, maxDx, maxDy, maxDTheta, totalTime);
            }
            else
            {
                Rotation2d diff = headings[1].rotateBy(headings[0].inverse());
                if (diff.getRadians() > math::PI)
                {
                    diff = diff.inverse().rotateBy(Rotation2d::fromRadians(math::PI));
                }
                Rotation2d interpolated_heading = headings[0].rotateBy(diff.times(t1 / totalTime));

                rv.push_back(trajectory::TrajectoryPoint<Pose2dWithCurvature, Rotation2d>{s.getPose2dWithCurvature(t1), interpolated_heading, (int)rv.size()-1});
            }
        }                                    

        std::vector<trajectory::TrajectoryPoint<Pose2dWithCurvature, Rotation2d>> SplineGenerator::parameterizeSpline(QuinticHermiteSpline &s,
                                                                                                                      std::vector<Rotation2d> &headings,
                                                                                                                      double maxDx,
                                                                                                                      double maxDy,
                                                                                                                      double maxDTheta,
                                                                                                                      double t0,
                                                                                                                      double t1)
        {
            std::vector<trajectory::TrajectoryPoint<Pose2dWithCurvature, Rotation2d>> rv;
            rv.push_back(trajectory::TrajectoryPoint<Pose2dWithCurvature, Rotation2d>{s.getPose2dWithCurvature(0.0), headings[0], 0});
            double dt = (t1 - t0);
            for (double t = 0; t < t1; t += dt / kMinSampleSize)
            {
                getSegmentArc(s, headings, rv, t, t + dt / kMinSampleSize, maxDx, maxDy, maxDTheta, dt);
            }
            return rv;
        }                                                                                                    
        std::vector<trajectory::TrajectoryPoint<Pose2dWithCurvature, Rotation2d>> SplineGenerator::parameterizeSpline(QuinticHermiteSpline &s, std::vector<Rotation2d> &headings)
        {
            return parameterizeSpline(s, headings, kMaxDX, kMaxDY, kMaxDTheta, 0.0, 1.0);
        }

        std::vector<trajectory::TrajectoryPoint<Pose2dWithCurvature, Rotation2d>> SplineGenerator::parameterizeSpline(QuinticHermiteSpline &s,
                                                                                                            std::vector<Rotation2d> &headings,
                                                                                                            double maxDx,
                                                                                                            double maxDy,
                                                                                                            double maxDTheta)
        {
            return parameterizeSpline(s, headings, maxDx, maxDy, maxDTheta, 0.0, 1.0);
        }


        std::vector<trajectory::TrajectoryPoint<Pose2dWithCurvature, Rotation2d>> SplineGenerator::parameterizeSplines(std::vector<QuinticHermiteSpline> &splines, std::vector<Rotation2d> &headings)
        {
            return parameterizeSplines(splines, headings, kMaxDX, kMaxDY, kMaxDTheta);
        }

        std::vector<trajectory::TrajectoryPoint<Pose2dWithCurvature, Rotation2d>> SplineGenerator::parameterizeSplines(std::vector<QuinticHermiteSpline> &splines,
                                                                                                                std::vector<Rotation2d> &headings,
                                                                                                                double maxDx,
                                                                                                                double maxDy,
                                                                                                                double maxDTheta)
        {
            std::vector<trajectory::TrajectoryPoint<Pose2dWithCurvature, Rotation2d>> rv;

            if (splines.empty())
            {
                return rv;
            }

            rv.push_back(trajectory::TrajectoryPoint<Pose2dWithCurvature, Rotation2d>{splines[0].getPose2dWithCurvature(0.0), headings[0].getRotation(), 0});

            for (size_t i = 0; i < splines.size(); i++)
            {
                QuinticHermiteSpline &s = splines[i];
                std::vector<Rotation2d> spline_rots;
                spline_rots.push_back(headings[i]);
                spline_rots.push_back(headings[i + 1]);

                std::vector<trajectory::TrajectoryPoint<Pose2dWithCurvature, Rotation2d>> samples = parameterizeSpline(s, spline_rots, maxDx, maxDy, maxDTheta);
                samples.erase(std::begin(samples));
                rv.insert(std::end(rv), std::begin(samples), std::end(samples));
            }

            return rv;
        }

        // void SplineGenerator::getSegmentArc(QuinticHermiteSpline &s, std::vector<ck::team254_geometry::Pose2dWithCurvature> &rv, double t0, double t1, double maxDx, double maxDy, double maxDTheta)
        // {
        //     ck::team254_geometry::Translation2d p0 = s.getPoint(t0);
        //     ck::team254_geometry::Translation2d p1 = s.getPoint(t1);
        //     ck::team254_geometry::Rotation2d r0 = s.getHeading(t0);
        //     ck::team254_geometry::Rotation2d r1 = s.getHeading(t1);
        //     ck::team254_geometry::Pose2d transformation(ck::team254_geometry::Translation2d(p0, p1).rotateBy(r0.inverse()), r1.rotateBy(r0.inverse()));
        //     ck::team254_geometry::Twist2d twist = ck::team254_geometry::Pose2d::log(transformation);
        //     if (twist.dy > maxDy || twist.dx > maxDx || twist.dtheta > maxDTheta)
        //     {
        //         getSegmentArc(s, rv, t0, (t0 + t1) / 2, maxDx, maxDy, maxDTheta);
        //         getSegmentArc(s, rv, (t0 + t1) / 2, t1, maxDx, maxDy, maxDTheta);
        //     }
        //     else
        //     {
        //         rv.push_back(s.getPose2dWithCurvature(t1));
        //     }
        // }

        // std::vector<ck::team254_geometry::Pose2dWithCurvature> SplineGenerator::parameterizeSpline(QuinticHermiteSpline &s, double maxDx, double maxDy, double maxDTheta, double t0, double t1)
        // {
        //     std::vector<ck::team254_geometry::Pose2dWithCurvature> rv;
        //     rv.push_back(s.getPose2dWithCurvature(0.0));
        //     double dt = (t1 - t0);
        //     for (double t = 0; t < t1; t += dt / kMinSampleSize)
        //     {
        //         getSegmentArc(s, rv, t, t + dt / kMinSampleSize, maxDx, maxDy, maxDTheta);
        //     }
        //     return rv;
        // }

        // std::vector<ck::team254_geometry::Pose2dWithCurvature> SplineGenerator::parameterizeSpline(QuinticHermiteSpline &s)
        // {
        //     return parameterizeSpline(s, kMaxDX, kMaxDY, kMaxDTheta, 0.0, 1.0);
        // }

        // std::vector<ck::team254_geometry::Pose2dWithCurvature> SplineGenerator::parameterizeSpline(QuinticHermiteSpline &s, double maxDx, double maxDy, double maxDTheta)
        // {
        //     return parameterizeSpline(s, maxDx, maxDy, maxDTheta, 0.0, 1.0);
        // }

        // std::vector<ck::team254_geometry::Pose2dWithCurvature> SplineGenerator::parameterizeSplines(std::vector<QuinticHermiteSpline> &splines)
        // {
        //     return parameterizeSplines(splines, kMaxDX, kMaxDY, kMaxDTheta);
        // }

        // std::vector<ck::team254_geometry::Pose2dWithCurvature> SplineGenerator::parameterizeSplines(std::vector<QuinticHermiteSpline> &splines, double maxDx, double maxDy, double maxDTheta)
        // {
        //     std::vector<ck::team254_geometry::Pose2dWithCurvature> rv;
        //     if (splines.empty())
        //     {
        //         return rv;
        //     }

        //     rv.push_back(splines[0].getPose2dWithCurvature(0.0));
        //     for (QuinticHermiteSpline spline : splines)
        //     {
        //         std::vector<ck::team254_geometry::Pose2dWithCurvature> samples = parameterizeSpline(spline, maxDx, maxDy, maxDTheta);
        //         samples.erase(samples.begin());

        //         rv.insert(rv.end(), samples.begin(), samples.end());
        //     }
        //     return rv;
        // }
    } // namespace team254_geometry
} // namespace ck