#include <vector>

#include "ck_utilities/geometry/geometry.hpp"
#include "ck_utilities/InterpolatingMap.hpp"

namespace SwerveTrajectory
{
    class BasicTrajectoryPoint
    {
    public:
        geometry::Pose pose;
        float speed;
    };

    class BasicTrajectory
    {
    public:
        std::vector<BasicTrajectoryPoint> points;
    };

    class DetailedTrajectoryPoint
    {
    public:
        DetailedTrajectoryPoint( BasicTrajectoryPoint basic_point)
        {
            this->pose = basic_point.pose;
            this->speed = basic_point.speed;
            this->desired_track = 0;
        }
        geometry::Pose pose;
        float desired_track;
        float speed;
        size_t associated_base_point;
    };

    class DetailedTrajectory
    {
    public:
        BasicTrajectory base_path;
        std::vector<DetailedTrajectoryPoint> points;
    };

    class SwerveTrajectorySmootherConfiguration
    {
    public:
        InterpolatingMap<float, float> track_acceleration_by_speed;
        InterpolatingMap<float, float> track_deceleration_by_speed;
        InterpolatingMap<float, float> track_turn_rate_by_speed;
        InterpolatingMap<float, float> heading_turn_rate_by_speed;
        float time_step_seconds;
    };

    class SwerveTrajectorySmoother
    {
    public:

        SwerveTrajectorySmoother (SwerveTrajectorySmootherConfiguration config)
        {
            this->config = config;
        }

        DetailedTrajectory smooth_path(BasicTrajectory trajectory);

    private:
        SwerveTrajectorySmootherConfiguration config;
        SwerveTrajectorySmoother() = delete;
    };
}