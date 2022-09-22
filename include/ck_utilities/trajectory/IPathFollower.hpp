#pragma once

#include "ck_utilities/team254_geometry/Pose2d.hpp"
#include "ck_utilities/team254_geometry/Twist2d.hpp"

namespace ck {
    namespace trajectory {
        class IPathFollower {
        public:
            virtual ck::team254_geometry::Twist2d steer(ck::team254_geometry::Pose2d current_pose) = 0;
            virtual bool isDone() = 0;
        };
    }
}