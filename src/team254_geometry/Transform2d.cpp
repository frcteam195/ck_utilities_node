#include "ck_utilities/team254_geometry/Transform2d.hpp"

namespace ck
{
    namespace team254_geometry
    {
        Transform2d::Transform2d(const Pose2d &initial, const Pose2d &last)
        {
            m_translation = (last.getTranslation() - initial.getTranslation())
                            .rotateBy(initial.getRotation().unaryMinus());
            m_rotation = last.getRotation().minus(initial.getRotation());
        }

        Transform2d::Transform2d(const Translation2d &translation, const Rotation2d &rotation)
            : m_translation(translation), m_rotation(rotation) {}

        Transform2d::Transform2d()
            : m_translation(), m_rotation() {}

        Transform2d Transform2d::times(double scalar) const 
        {
            return Transform2d(m_translation.times(scalar), m_rotation.times(scalar));
        }

        Transform2d Transform2d::plus(const Transform2d &other) const
        {
            return Transform2d(Pose2d(), Pose2d().transformBy(*this).transformBy(other));
        }

        Translation2d Transform2d::getTranslation() const
        {
            return m_translation;
        }

        double Transform2d::getX() const
        {
            return m_translation.x();
        }

        double Transform2d::getY() const 
        {
            return m_translation.y();
        }

        Rotation2d Transform2d::getRotation() const
        {
            return m_rotation;
        }
        Transform2d Transform2d::inverse() const
        {
            return Transform2d(
                getTranslation().unaryMinus().rotateBy(getRotation().unaryMinus()), getRotation().unaryMinus());
            
        }
            
        

    } // namespace team254_geometry
} // namespace ck
