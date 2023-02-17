#pragma once

#include "ck_utilities/team254_geometry/Translation2d.hpp"
#include "ck_utilities/team254_geometry/Rotation2d.hpp"
#include "ck_utilities/team254_geometry/Transform2d.hpp"
#include "ck_utilities/team254_geometry/Twist2d.hpp"
#include "ck_utilities/team254_geometry/IPose2d.hpp"

#include <cmath>

namespace ck
{
    namespace team254_geometry
    {
        class Transform2d;
        class Pose2dWithCurvature;

        class Pose2d : public IPose2d<Pose2d>
        {
        protected:
            Translation2d translation;
            Rotation2d rotation;

        public:
            static const Pose2d &identity();

            Pose2d();
            Pose2d(double x, double y, const Rotation2d &rotation);
            Pose2d(const Translation2d &translation, const Rotation2d &rotation);
            Pose2d(const Pose2dWithCurvature &other);

            bool operator==(const Pose2d &obj) const;
            friend std::ostream &operator<<(std::ostream &os, const Pose2d &t2d);

            static Pose2d fromTranslation(const Translation2d &translation);
            static Pose2d fromRotation(const Rotation2d &rotation);

            /**
             * Obtain a new Pose2d from a (constant curvature) velocity. See:
             * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
             */
            static Pose2d exp(const Twist2d &delta);

            /**
             * Logical inverse of the above.
             */
            static Twist2d log(const Pose2d &transform);

            Translation2d getTranslation() const override;
            Rotation2d getRotation() const override;
            Pose2d rotateBy(const Rotation2d &other) const override;
            Pose2d transformBy(const Pose2d &other) const override;
            Pose2d transformBy(const Transform2d &other) const;

            Pose2d add(const Pose2d &other) const override;
            Transform2d minus(const Pose2d &other) const;
            Pose2d relativeTo(const Pose2d &other) const;

            Pose2d inverse() const;
            Pose2d normal() const;
            Translation2d intersection(const Pose2d &other) const;
            bool isColinear(const Pose2d &other) const;
            bool epsilonEquals(const Pose2d &other, double epsilon) const;
            static Translation2d intersectionInternal(const Pose2d &a, const Pose2d &b);
            Pose2d interpolate(const Pose2d &other, double x) const override;
            double distance(const Pose2d &other) const override;
            bool equals(const Pose2d &other) override;
            Pose2d getPose() const override;
            Pose2d mirror() const override;

            std::string to_string() const;
        };
    } // namespace team254_geometry
} // namespace ck