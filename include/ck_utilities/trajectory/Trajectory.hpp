#pragma once

#include "ck_utilities/team254_geometry/State.hpp"
#include "IndexView.hpp"

#include <cmath>
#include <limits>
#include <type_traits>
#include <vector>

namespace ck
{
    namespace trajectory
    {
        template <class S, class T>
        class Trajectory
        {
            static_assert(std::is_base_of<ck::team254_geometry::State<S>, S>::value, "S must inherit from State<S>");
            static_assert(std::is_base_of<ck::team254_geometry::State<T>, T>::value, "T must inherit from State<T>");

        protected:
            std::vector<TrajectoryPoint<S, T>> points_;
            IndexView<S, T> index_view_;
            double default_velocity_;

        public:
            Trajectory()
                : points_()
            {
                index_view_.initialize(this);
            }

            Trajectory(std::vector<S> &states, std::vector<T> &headings)
                : points_()
            {
                index_view_.initialize(this);
                for (int i = 0; i < (int)states.size(); ++i)
                {
                    points_.push_back(TrajectoryPoint<S, T>{states[i], headings[i], i});
                }
            }

            Trajectory(const std::vector<TrajectoryPoint<S, T>> &points) : points_()
            {
                index_view_.initialize(this);
                for (int i = 0; i < (int)points.size(); i++)
                {
                    points_.push_back(TrajectoryPoint<S, T>{points[i].state_, points[i].heading_, i});
                }
            }

            Trajectory( const Trajectory& in ) : Trajectory() {
                points_ = in.points_;
            }

            void setDefaultVelocity(double velocity)
            {
                default_velocity_ = velocity;
            }

            double getDefaultVelocity() const
            {
                return default_velocity_;
            }

            Trajectory& operator=( const Trajectory& in ){
                points_ = in.points_;
                return *this;
            }

            bool isEmpty() {return points_.empty();}
            int length() { return points_.size(); }
            TrajectoryPoint<S, T> getPoint(int index) { return points_[index]; }
            S getState(int index) { return points_[index].state_; }
            T getHeading(int index) { return points_[index].heading_; }
            S getFirstState() { return getState(0); }
            S getLastState() { return getState(length() - 1); }

            TrajectorySamplePoint<S, T> getInterpolated(double index)
            {
                if (isEmpty())
                {
                    //TODO: Check if this is okay instead of returning null
                    throw;
                }
                else if (index <= 0.0)
                {
                    return TrajectorySamplePoint<S, T>(points_[0]);
                }
                else if (index >= length() - 1)
                {
                    return TrajectorySamplePoint<S, T>(points_[length() - 1]);
                }
                int i = (int)std::floor(index);
                double frac = index - i;
                if (frac <= std::numeric_limits<double>::min())
                {
                    return TrajectorySamplePoint<S, T>(points_[i]);
                }
                else if (frac >= 1.0 - std::numeric_limits<double>::min())
                {
                    return TrajectorySamplePoint<S, T>(points_[i + 1]);
                }
                else
                {
                    return TrajectorySamplePoint<S, T> { getState(i).interpolate(getState(i + 1), frac),
                                                         getHeading(i).interpolate(getHeading(i + 1), frac),
                                                         i,
                                                         i + 1 };
                }
            }
            IndexView<S, T>& getIndexView() { return index_view_; }
        };
    } // namespace trajectory
} // namespace ck
