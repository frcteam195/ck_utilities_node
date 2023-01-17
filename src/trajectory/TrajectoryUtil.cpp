#include "ck_utilities/trajectory/TrajectoryUtil.hpp"

namespace ck
{
    namespace trajectory
    {
        // Trajectory<team254_geometry::Pose2dWithCurvature> TrajectoryUtil::trajectoryFromSplines(std::vector<team254_geometry::QuinticHermiteSpline> splines,
        //                                                                                 double maxDx,
        //                                                                                 double maxDy,
        //                                                                                 double maxDtheta)
        // {
        //     std::vector<ck::team254_geometry::Pose2dWithCurvature> splinePoses = team254_geometry::SplineGenerator::parameterizeSplines(splines, maxDx, maxDy, maxDtheta);
        //     return Trajectory<team254_geometry::Pose2dWithCurvature>(splinePoses);
        // }

        // Trajectory<team254_geometry::Pose2dWithCurvature> TrajectoryUtil::trajectoryFromSplineWaypoints(std::vector<team254_geometry::Pose2d> waypoints,
        //                                                                                         double maxDx,
        //                                                                                         double maxDy,
        //                                                                                         double maxDtheta)
        // {
        //     std::vector<team254_geometry::QuinticHermiteSpline> splines;

        //     for (size_t i = 1; i < waypoints.size(); ++i)
        //     {
        //         splines.push_back(team254_geometry::QuinticHermiteSpline(waypoints[i - 1], waypoints[i]));
        //     }

        //     team254_geometry::QuinticHermiteSpline::optimizeSpline(splines);

        //     return trajectoryFromSplines(splines, maxDx, maxDy, maxDtheta);
        // }

        Trajectory<team254_geometry::Pose2dWithCurvature, team254_geometry::Rotation2d> TrajectoryUtil::trajectoryFromWaypoints(std::vector<team254_geometry::Pose2d> &waypoints,
                                                                                                                                std::vector<team254_geometry::Rotation2d> &headings,
                                                                                                                                double maxDx,
                                                                                                                                double maxDy,
                                                                                                                                double maxDTheta)
        {
            std::vector<team254_geometry::QuinticHermiteSpline> splines;
            for (size_t i = 1; i < waypoints.size(); i++)
            {
                splines.push_back(team254_geometry::QuinticHermiteSpline(waypoints[i - 1], waypoints[i]));
            }

            team254_geometry::QuinticHermiteSpline::optimizeSpline(splines);

            return trajectoryFromSplinesAndHeadings(splines, headings, maxDx, maxDy, maxDTheta);
        }

        Trajectory<team254_geometry::Pose2dWithCurvature, team254_geometry::Rotation2d> TrajectoryUtil::trajectoryFromSplinesAndHeadings(std::vector<team254_geometry::QuinticHermiteSpline> &splines,
                                                                                                                                         std::vector<team254_geometry::Rotation2d> &headings,
                                                                                                                                         double maxDx,
                                                                                                                                         double maxDy,
                                                                                                                                         double maxDTheta)
        {
            std::vector<TrajectoryPoint<team254_geometry::Pose2dWithCurvature, team254_geometry::Rotation2d>> splineTrajPoints = team254_geometry::SplineGenerator::parameterizeSplines(splines, headings, maxDx, maxDy, maxDTheta);
            return Trajectory<team254_geometry::Pose2dWithCurvature, team254_geometry::Rotation2d>(splineTrajPoints);
        }                                                                                                                                

    } // namespace trajectory

} // namespace ck