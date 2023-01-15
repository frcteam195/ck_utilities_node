#pragma once

#include "ck_utilities/team254_geometry/Pose2dWithCurvature.hpp"
#include "ck_utilities/team254_geometry/QuinticHermiteSpline.hpp"
#include "ck_utilities/team254_geometry/SplineGenerator.hpp"
#include "ck_utilities/trajectory/Trajectory.hpp"
#include "ck_utilities/trajectory/timing/TimedState.hpp"

namespace ck
{
    namespace trajectory
    {
        class TrajectoryUtil
        {
        public:
            template <class S, class T>
            static Trajectory<S, T> mirror(Trajectory<S, T> trajectory)
            {
                std::vector<S> waypoints;
                std::vector<T> headings;

                for (int i = 0; i < trajectory.length(); ++i)
                {
                    waypoints.push_back(trajectory.getState(i).mirror());
                    headings.push_back(trajectory.getHeading(i).mirror());
                }

                return Trajectory<S, T>(waypoints, headings);
            }

            template <class S, class T>
            static Trajectory<timing::TimedState<S>, timing::TimedState<T>> mirrorTimed(Trajectory<timing::TimedState<S>, timing::TimedState<T>> trajectory)
            {
                std::vector<timing::TimedState<S>> waypoints;
                std::vector<timing::TimedState<T>> headings;

                for (int i = 0; i < trajectory.length(); ++i)
                {
                    timing::TimedState<S> timedState = trajectory.getState(i);
                    waypoints.push_back(timing::TimedState(timedState.state().mirror(), timedState.t(), timedState.velocity(), timedState.acceleration()));
                    headings.push_back(timing::TimedState<T>(trajectory.getHeading(i).state().mirror()));
                }

                return Trajectory<timing::TimedState<S>, timing::TimedState<T>>(waypoints, headings);
            }

            template <class S, class T>
            static Trajectory<S, T> transform(const Trajectory<S, T> &trajectory, const team254_geometry::Pose2d &tranform)
            {
                std::vector<S> waypoints;
                std::vector<T> headings;
                for (int i = 0; i < trajectory.length(); i++)
                {
                    waypoints.push_back(trajectory.getState(i).transformBy(transform));
                    headings.push_back(trajectory.getHeading(i).rotateBy(transform));
                }

                return Trajectory<S, T>(waypoints, headings);
            }

            template <class S, class T>
            static Trajectory<S, T> resample(const TrajectoryView<S, T> &trajectory_view, double interval)
            {
                if (interval <= math::kEpsilon)
                {
                    return Trajectory<S, T>();
                }

                const int num_states = (int)std::ceil((trajectory_view.last_interpolant() - trajectory_view.first_interpolant()) / interval);
                std::vector<S> states;
                std::vector<T> headings;

                for (int i = 0; i < num_states; i++)
                {
                    states.add(trajectory_view.sample(i * interval + trajectory_view.first_interpolant()).state());
                    headings.add(trajectory_view.sample(i * interval + trajectory_view.first_interpolant()).heading());
                }

                return Trajectory<S, T>(states, headings);
            } 

            static Trajectory<team254_geometry::Pose2dWithCurvature, team254_geometry::Rotation2d> trajectoryFromWaypoints(std::vector<team254_geometry::Pose2d> &waypoints,
                                                                                                                           std::vector<team254_geometry::Rotation2d> &headings,
                                                                                                                           double maxDx,
                                                                                                                           double maxDy,
                                                                                                                           double maxDTheta);

            static Trajectory<team254_geometry::Pose2dWithCurvature, team254_geometry::Rotation2d> trajectoryFromSplinesAndHeadings(std::vector<team254_geometry::QuinticHermiteSpline> &splines,
                                                                                                                                    std::vector<team254_geometry::Rotation2d> &headings,
                                                                                                                                    double maxDx,
                                                                                                                                    double maxDy,
                                                                                                                                    double maxDTheta);

            // static Trajectory<team254_geometry::Pose2dWithCurvature> trajectoryFromSplines(std::vector<team254_geometry::QuinticHermiteSpline> splines,
            //                                                                        double maxDx,
            //                                                                        double maxDy,
            //                                                                        double maxDtheta);

            // static Trajectory<team254_geometry::Pose2dWithCurvature> trajectoryFromSplineWaypoints(std::vector<team254_geometry::Pose2d> waypoints,
            //                                                                                double maxDx,
            //                                                                                double maxDy,
            //                                                                                double maxDtheta);
        };

    } // namespace trajectory

} // namespace ck