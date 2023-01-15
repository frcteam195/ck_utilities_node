#pragma once

#include <vector>
#include "ck_utilities/team254_geometry/Geometry.hpp"
#include "TrajectoryIterator.hpp"
#include "IPathFollower.hpp"
#include "ck_utilities/CKMath.hpp"
#include "Arc.hpp"

namespace ck
{
    namespace trajectory
    {
        template <class S, class T>
        class PurePursuitController : public IPathFollower
        {
            static_assert(std::is_base_of<ck::team254_geometry::ITranslation2d<S>, S>::value, "S must inherit from ITranslation2d<S>");
            static_assert(std::is_base_of<ck::team254_geometry::IRotation2d<T>, T>::value, "T must inherit from IRotation2d<T>");

        protected:
            TrajectoryIterator<S, T> iterator_;
            double sampling_dist_;
            double lookahead_;
            double goal_tolerance_;
            bool done_ = false;

        public:
            PurePursuitController(const DistanceView<S, T> &path, double sampling_dist, double lookahead, double goal_tolerance)
                : iterator_(path), sampling_dist_(sampling_dist), lookahead_(lookahead), goal_tolerance_(goal_tolerance) {}

            ck::team254_geometry::Twist2d steer(const ck::team254_geometry::Pose2d &current_pose)
            {
                done_ = done_ || (iterator_.isDone() && current_pose.getTranslation().distance(iterator_.getState().getTranslation()) <= goal_tolerance_);
                if (done_)
                {
                    return Twist2d(0.0, 0.0, 0.0);
                }

                double remaining_progress = iterator_.getRemainingProgress();
                double goal_progress = 0.0;
                // Find the first point > lookahead distance away from current_pose, or the last point otherwise.
                for (double progress = 0.0; progress <= remaining_progress; progress = ck::math::min(remaining_progress, progress + sampling_dist_))
                {
                    double dist = current_pose.getTranslation().distance(iterator_.preview(progress).state().getTranslation());
                    if (dist > lookahead_)
                    {
                        if (goal_progress == 0.0 && !iterator_.isDone())
                        {
                            // Make sure we don't get stuck due to numerical issues when sampling dist is large relative to
                            // lookahead.
                            goal_progress = progress;
                        }
                        break;
                    }
                    goal_progress = progress;
                    if (progress == remaining_progress)
                    {
                        break;
                    }
                }
                iterator_.advance(goal_progress);
                const team254_geometry::Translation2d path_setpoint = current_pose.getTranslation().inverse().translateBy(iterator_.getState().getTranslation());
                const team254_geometry::Rotation2d heading_setpoint = current_pose.getRotation().inverse().rotateBy(iterator_.getHeading().getRotation());
                if (path_setpoint.norm() < math::kEpsilon)
                {
                    return team254_geometry::Twist2d(0.0, 0.0, 0.0);
                }
                else
                {
                    return team254_geometry::Twist2d(path_setpoint.x(), path_setpoint.y(), heading_setpoint.getRadians());
                }
            }

            bool isDone() { return done_; }

            static double getDirection(Pose2d pose, S point)
            {
                ck::team254_geometry::Translation2d poseToPoint(pose.getTranslation(), point.getTranslation());
                ck::team254_geometry::Translation2d robot = pose.getRotation().toTranslation();
                double cross = robot.x() * poseToPoint.y() - robot.y() * poseToPoint.x();
                return (cross < 0.) ? -1. : 1.; // if robot < pose turn left
            }
        };
    } // namespace trajectory
} // namespace ck