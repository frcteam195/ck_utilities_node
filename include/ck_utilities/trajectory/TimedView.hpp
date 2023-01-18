#pragma once

#include "ck_utilities/CKMath.hpp"
#include "ck_utilities/team254_geometry/State.hpp"
#include "TrajectoryView.hpp"
#include "TrajectorySamplePoint.hpp"
#include "timing/TimedState.hpp"

#include <vector>

namespace ck
{
    namespace trajectory
    {
        template <class S, class T>
        class TimedView : public TrajectoryView<timing::TimedState<S>, timing::TimedState<T>>
        {
            static_assert(std::is_base_of<ck::team254_geometry::State<S>, S>::value, "S must inherit from State<S>");
            static_assert(std::is_base_of<ck::team254_geometry::State<T>, T>::value, "T must inherit from State<T>");

        protected:
            Trajectory<timing::TimedState<S>, timing::TimedState<T>> *trajectory_;
            double start_t_;
            double end_t_;

        public:
            TimedView() {}

            TimedView(Trajectory<timing::TimedState<S>, timing::TimedState<T>> &traj)
                : trajectory_(&traj),
                  start_t_(traj.getState(0).t()),
                  end_t_(traj.getState(traj.length() - 1).t())
            {
                
            }

            double first_interpolant() const override { return start_t_; }
            double last_interpolant() const override { return end_t_; }

            /*
            TrajectorySamplePoint<ck::trajectory::timing::TimedState<S>> sample(double t) override
            {
                return TrajectorySamplePoint<ck::trajectory::timing::TimedState<S>>(
                    trajectory_->getPoint(trajectory_->length() - 1)
                    );
            }
            */

            TrajectorySamplePoint<timing::TimedState<S>, timing::TimedState<T>> sample(double t) override
            {
                if (t >= end_t_)
                {
                    return TrajectorySamplePoint<timing::TimedState<S>, timing::TimedState<T>>(trajectory_->getPoint(trajectory_->length() - 1));
                }
                if (t <= start_t_)
                {
                    return TrajectorySamplePoint<timing::TimedState<S>, timing::TimedState<T>>(trajectory_->getPoint(0));
                }
                for (int i = 1; i < trajectory_->length(); ++i)
                {
                    TrajectoryPoint<timing::TimedState<S>, timing::TimedState<T>> s = trajectory_->getPoint(i);
                    if (s.state_.t() >= t)
                    {
                        TrajectoryPoint<timing::TimedState<S>, timing::TimedState<T>> prev_s = trajectory_->getPoint(i - 1);
                        if (ck::math::epsilonEquals(s.state_.t(), prev_s.state_.t()))
                        {
                            return TrajectorySamplePoint<timing::TimedState<S>, timing::TimedState<T>>(s);
                        }
                        // return TrajectorySamplePoint<timing::TimedState<S>, timing::TimedState<T>>(prev_s.state_.interpolate(s.state_, (t - prev_s.state_.t()) / (s.state_.t() - prev_s.state_.t())), i - 1, i);
                        return TrajectorySamplePoint<timing::TimedState<S>, timing::TimedState<T>>(prev_s.state_.interpolate(s.state_,
                                                                                                                 (t - prev_s.state_.t()) / (s.state_.t() - prev_s.state_.t())),
                                                                                                   prev_s.heading_.interpolate(s.heading_,
                                                                                                                   (t - prev_s.heading_.t()) / (s.heading_.t() - prev_s.heading_.t())),
                                                                                                   i - 1, i);
                    }
                }
                throw;
                // throw new RuntimeException();
            }

            Trajectory<timing::TimedState<S>, timing::TimedState<T>> trajectory() override { return *trajectory_; }
        };
    } // namespace trajectory
} // namespace ck
