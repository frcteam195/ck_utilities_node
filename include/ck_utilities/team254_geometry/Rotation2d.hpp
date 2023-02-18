#pragma once

#include "ck_utilities/CKMath.hpp"
#include "ck_utilities/team254_geometry/IRotation2d.hpp"
#include "ck_utilities/team254_geometry/Translation2d.hpp"

#include <cmath>
#include <limits>

namespace ck
{
    namespace team254_geometry
    {
        class Translation2d;

        class Rotation2d : public IRotation2d<Rotation2d>
        {
        protected:
            double cos_angle;
            double sin_angle;
            double radians_;

            double WrapRadians(double radians) const;

        public:
            static const Rotation2d &identity();
            static const Rotation2d &kPi();
            static const Rotation2d &kHalfPi();
            Rotation2d();
            Rotation2d(double x, double y, bool normalize);
            Rotation2d(double radians, bool normalize);
            Rotation2d(double x, double y, double radians);
            Rotation2d(const Translation2d &direction, bool normalize);

            bool operator==(const Rotation2d &obj) const;
            friend std::ostream &operator<<(std::ostream &os, const Rotation2d &r2d);

            static Rotation2d fromRadians(double angle_radians);
            static Rotation2d fromDegrees(double angle_degrees);

            double cos() const;
            double sin() const;
            double tan() const;
            double getRadians() const;
            double getDegrees() const;
            Rotation2d unaryMinus() const;
            Rotation2d minus(const Rotation2d &other) const;
            Rotation2d times(double scalar) const;
            Rotation2d getRotation() const override;
            Rotation2d rotateBy(const Rotation2d &other) const override;
            Rotation2d mirror() const override;
            Rotation2d normal() const;
            Rotation2d inverse() const;
            Rotation2d flip() const;
            bool isParallel(const Rotation2d &other) const;
            Translation2d toTranslation() const;
            Rotation2d interpolate(const Rotation2d &other, double interpFactor) const override;
            double distance(const Rotation2d &other) const override;
            bool equals(const Rotation2d &other) override;

            Rotation2d add(const Rotation2d &other) const override;

        private:
            void calcRadians(void);
            void calcTrig(void);
        };
    } // namespace team254_geometry
} // namespace ck