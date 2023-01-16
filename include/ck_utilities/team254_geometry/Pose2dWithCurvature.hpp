#pragma once

#include "ck_utilities/CKMath.hpp"
#include "ck_utilities/team254_geometry/Pose2d.hpp"
#include "ck_utilities/team254_geometry/IPose2d.hpp"
#include "ck_utilities/team254_geometry/ICurvature.hpp"

namespace ck
{
    namespace team254_geometry
    {
        class Pose2dWithCurvature : public IPose2d<Pose2dWithCurvature>, public ICurvature<Pose2dWithCurvature>
        {
        protected:
            Pose2d pose;
            double curvature;
            double dcurvature_ds;

        public:
            static const Pose2dWithCurvature &identity();

            Pose2dWithCurvature();
            Pose2dWithCurvature(const Pose2d &pose, double curvature);
            Pose2dWithCurvature(const Pose2d &pose, double curvature, double dcurvature_ds);
            Pose2dWithCurvature(const Translation2d &translation, const Rotation2d &rotation, double curvature);
            Pose2dWithCurvature(const Translation2d &translation, const Rotation2d &rotation, double curvature, double dcurvature_ds = 0);
            virtual ~Pose2dWithCurvature() {};

            bool operator==(const Pose2dWithCurvature &obj) const;
            friend std::ostream &operator<<(std::ostream &os, const Pose2dWithCurvature &t2d);

            Pose2d getPose() const override;
            Pose2dWithCurvature transformBy(const Pose2d &transform) const override;
            Pose2dWithCurvature mirror() const override;
            double getCurvature() const override;
            double getDCurvatureDs() const override;
            Translation2d getTranslation() const override;
            Rotation2d getRotation() const override;
            Pose2dWithCurvature interpolate(const Pose2dWithCurvature &other, double interpFactor) const override;
            double distance(const Pose2dWithCurvature &other) const override;
            bool equals(const Pose2dWithCurvature &other) override;

            Pose2dWithCurvature rotateBy(const Rotation2d &rotation) const override;
            Pose2dWithCurvature add(const Pose2dWithCurvature &pose) const override;
        };
    } // namespace team254_geometry
} // namespace ck