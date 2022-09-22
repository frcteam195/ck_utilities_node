#pragma once

#include "ck_utilities/team254_geometry/IRotation2d.hpp"
#include "ck_utilities/team254_geometry/ITranslation2d.hpp"

namespace ck
{
    namespace team254_geometry
    {
        class Pose2d;

        template <class S>
        class IPose2d : public ITranslation2d<S>, public IRotation2d<S>
        {
        public:
            virtual Pose2d getPose() const = 0;
            virtual S transformBy(const Pose2d &transform) const = 0;
            virtual S mirror() const = 0;
        };
    } // namespace team254_geometry
} // namespace ck