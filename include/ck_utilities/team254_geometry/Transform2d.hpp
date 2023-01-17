#pragma once

#include "ck_utilities/team254_geometry/Pose2d.hpp"

namespace ck
{
    namespace team254_geometry
    {
        class Pose2d;

        class Transform2d
        {
        public:
            Transform2d(const Pose2d &initial, const Pose2d &last);
            Transform2d(const Translation2d &translation, const Rotation2d &rotation);
            Transform2d();
            Transform2d times(double scalar) const;
            Transform2d plus(const Transform2d &other) const;
            Translation2d getTranslation() const;
            double getX() const;
            double getY() const;
            Rotation2d getRotation() const;
            Transform2d inverse() const;
            //bool equal(Object obj);
        private:
            Translation2d m_translation;
            Rotation2d m_rotation;
        };
    }
}