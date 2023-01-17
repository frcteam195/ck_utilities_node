#pragma once

#include <vector>
#include <cmath>
#include <type_traits>
#include "ck_utilities/team254_geometry/State.hpp"
#include "ck_utilities/CKMath.hpp"
#include "TrajectoryView.hpp"

namespace ck
{
    namespace trajectory
    {
        template <class S, class T>
        class DistanceView : public TrajectoryView<S, T>
        {
            static_assert(std::is_base_of<ck::team254_geometry::State<S>, S>::value, "S must inherit from State<S>");
            static_assert(std::is_base_of<ck::team254_geometry::State<T>, T>::value, "T must inherit from State<T>");

        protected:
            Trajectory<S, T> *trajectory_;
            std::vector<double> distances_;

        public:
            DistanceView(Trajectory<S, T> &trajectory) : trajectory_(&trajectory), distances_(trajectory.length())
            {
                distances_[0] = 0.0;
                for (int i = 1; i < trajectory_->length(); ++i)
                {
                    distances_[i] = distances_[i - 1] + trajectory_->getState(i - 1).distance(trajectory_->getState(i));
                }
            }

            TrajectorySamplePoint<S, T> sample(double distance) override
            {
                if (distance >= last_interpolant())
                    return TrajectorySamplePoint<S, T>(trajectory_->getPoint(trajectory_->length() - 1));
                if (distance <= 0.0)
                    return TrajectorySamplePoint<S, T>(trajectory_->getPoint(0));
                for (size_t i = 1; i < distances_.size(); ++i)
                {
                    TrajectoryPoint<S, T> s = trajectory_->getPoint(i);
                    if (distances_[i] >= distance)
                    {
                        TrajectoryPoint<S, T> prev_s = trajectory_->getPoint(i - 1);
                        if (ck::math::epsilonEquals(distances_[i], distances_[i - 1]))
                        {
                            return TrajectorySamplePoint<S, T>(s);
                        }
                        else
                        {
                            // return TrajectorySamplePoint<S, T>(prev_s.state_.interpolate(s.state_, (distance - distances_[i - 1]) / (distances_[i] - distances_[i - 1])), i - 1, i);
                            return TrajectorySamplePoint<S, T>(prev_s.state_.interpolate(s.state_,
                                                                                         (distance - distances_[i - 1]) / (distances_[i] - distances_[i - 1])),
                                                               prev_s.heading_.interpolate(s.heading_,
                                                                                           (distance - distances_[i - 1]) / (distances_[i] - distances_[i - 1])),
                                                               i - 1, i);
                        }
                    }
                }
                throw;
            }

            double last_interpolant() const override
            {
                return distances_[distances_.size() - 1];
            }

            double first_interpolant() const override
            {
                return 0.0;
            }

            Trajectory<S, T> trajectory() override
            {
                return *trajectory_;
            }
        };
    } // namespace trajectory
} // namespace ck