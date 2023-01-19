#pragma once

#include "ck_utilities/CKMath.hpp"
#include "ck_utilities/team254_geometry/State.hpp"
#include "TrajectoryView.hpp"
#include "TrajectorySamplePoint.hpp"

#include <type_traits>

namespace ck
{
    namespace trajectory
    {
        template <class S, class T>
        class TrajectoryIterator
        {
            static_assert(std::is_base_of<ck::team254_geometry::State<S>, S>::value, "S must inherit from State");
            static_assert(std::is_base_of<ck::team254_geometry::State<T>, T>::value, "T must inherit from State");

        protected:
            TrajectoryView<S, T> *view_;
            double progress_;
            TrajectorySamplePoint<S, T> *current_sample_;

        public:

            TrajectoryIterator(TrajectoryView<S, T> *view)
                {
                    view_ = view;
                    progress_ = view->first_interpolant();
                    current_sample_ = new TrajectorySamplePoint<S, T>( view->sample(view->first_interpolant()) );
                }

            TrajectoryIterator()
                {
                    view_ = nullptr;
                    progress_ = 0.0;
                    current_sample_ = nullptr;
                }

            bool isDone()
                {
                    return getRemainingProgress() == 0.0;
                }

            double getProgress()
                {
                    return progress_;
                }

            double getRemainingProgress()
                {
                    return ck::math::max(0.0, view_->last_interpolant() - progress_);
                }

            TrajectorySamplePoint<S, T> getSample()
                {
                    return *current_sample_;
                }

            S getState()
                {
                    return (current_sample_->state());
                }

            T getHeading()
                {
                    return (current_sample_->heading());
                }

            TrajectorySamplePoint<S, T> advance(double additional_progress)
                {
                    progress_ = ck::math::max(view_->first_interpolant(), ck::math::min(view_->last_interpolant(), progress_ + additional_progress));
                    *current_sample_ = view_->sample(progress_);
                    return *current_sample_;
                }

            TrajectorySamplePoint<S, T> preview(double additional_progress)
                {
                    double progress = ck::math::max(view_->first_interpolant(), ck::math::min(view_->last_interpolant(), progress_ + additional_progress));
                    return view_->sample(progress);
                }
            Trajectory<S, T> trajectory() { return view_->trajectory(); }
        };
    } // namespace trajectory
} // namespace ck
