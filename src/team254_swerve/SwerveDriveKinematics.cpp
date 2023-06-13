#include "ck_utilities/team254_swerve/SwerveDriveKinematics.hpp"

#include <iostream>

using namespace ck::planners;
using namespace ck::team254_geometry;

namespace ck
{
    namespace team254_swerve
    {
        SwerveDriveKinematics::SwerveDriveKinematics(std::vector<Translation2d> wheelsMeters)
        {
            if (wheelsMeters.size() < 2)
            {
                throw;
            }
            m_numModules = wheelsMeters.size();
            m_modules = wheelsMeters;
            m_inverseKinematics.resize(m_numModules * 2, 3);
            m_rotations.reserve(m_numModules);

            for (int i = 0; i < m_numModules; i++)
            {
                m_inverseKinematics.row(i * 2 + 0) << 1, 0, -m_modules.at(i).y();
                m_inverseKinematics.row(i * 2 + 1) << 0, 1, m_modules.at(i).x();
                m_rotations.push_back(Rotation2d(m_modules.at(i).x(), m_modules.at(i).y(), true));
            }
            m_forwardKinematics = m_inverseKinematics.completeOrthogonalDecomposition().pseudoInverse();
        }

        std::vector<SwerveModuleState> SwerveDriveKinematics::toSwerveModuleStates(
            ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters)
        {
            if (!centerOfRotationMeters.equals(m_prevCoR))
            {
                for (int i = 0; i < m_numModules; i++)
                {
                    m_inverseKinematics.row(i * 2 + 0) <<
                        1,
                        0,
                        -m_modules.at(i).y() + centerOfRotationMeters.y();
                    m_inverseKinematics.row(i * 2 + 1) <<
                        0,
                        1,
                        m_modules.at(i).x() - centerOfRotationMeters.x();
                }
            }
            m_prevCoR = centerOfRotationMeters;

            Eigen::Vector3d chassisSpeedsVector;
            chassisSpeedsVector <<
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond;
            
            auto moduleStatesMatrix = m_inverseKinematics * chassisSpeedsVector;
            std::vector<SwerveModuleState> moduleStates;

            for (int i = 0; i < m_numModules; i++)
            {
                std::cout << "did x " << i << std::endl;
                std::cout << "did y " << i << std::endl;

                double speed = math::hypotenuse(x, y);
                Rotation2d angle(x, y, true);

                moduleStates.push_back(SwerveModuleState(speed, angle));
            }

            return moduleStates;
        }

        std::vector<SwerveModuleState> SwerveDriveKinematics::toSwerveModuleStates(ChassisSpeeds chassisSpeeds)
        {
            return toSwerveModuleStates(chassisSpeeds, Translation2d());
        }

        ChassisSpeeds SwerveDriveKinematics::toChassisSpeeds(std::vector<SwerveModuleState> wheelStates)
        {
            if ((int)wheelStates.size() != m_numModules)
            {
                throw;
            }
            Eigen::MatrixXd moduleStatesMatrix(m_numModules * 2, 1);

            for (int i = 0; i < m_numModules; i++)
            {
                SwerveModuleState module = wheelStates.at(i);
                moduleStatesMatrix.row(i * 2) << module.speedMetersPerSecond * module.angle.cos();
                moduleStatesMatrix.row(i * 2 + 1) << module.speedMetersPerSecond * module.angle.sin();
            }

            auto chassisSpeedsVector = m_forwardKinematics * moduleStatesMatrix;
            return ChassisSpeeds(
                chassisSpeedsVector(0, 0),
                chassisSpeedsVector(1, 0),
                chassisSpeedsVector(2, 0));
        }

        ChassisSpeeds SwerveDriveKinematics::toChassisSpeedWheelConstraints(std::vector<SwerveModuleState> wheelStates)
        {
            if ((int)wheelStates.size() != m_numModules)
            {
                throw;
            }

            Eigen::MatrixXd constrainsMatrix(m_numModules * 2, 3);
            for (int i = 0; i < m_numModules; i++)
            {
                SwerveModuleState module = wheelStates.at(i);

                auto beta = module.angle.rotateBy(
                    m_rotations.at(i).inverse()).rotateBy(Rotation2d::fromRadians(math::PI / 2.0));

                constrainsMatrix.row(i * 2) <<
                    module.angle.cos(),
                    module.angle.sin(),
                    -m_modules.at(i).norm() * beta.cos();
                constrainsMatrix.row(i * 2 + 1) <<
                    -module.angle.sin(),
                    module.angle.cos(),
                    m_modules.at(i).norm() * beta.sin();
            }

            auto pseudoInv = constrainsMatrix.completeOrthogonalDecomposition().pseudoInverse();

            Eigen::MatrixXd enforcedConstraints(m_numModules * 2, 1);

            for (int i = 0; i < m_numModules; i++)
            {
                enforcedConstraints.row(i*2) << wheelStates.at(i).speedMetersPerSecond;
                enforcedConstraints.row(i*2+1) << 0;
            }

            auto chassisSpeedsVector = pseudoInv * enforcedConstraints;

            return ChassisSpeeds(
                chassisSpeedsVector(0, 0),
                chassisSpeedsVector(1, 0),
                chassisSpeedsVector(2, 0));
        }

        void SwerveDriveKinematics::desaturateWheelSpeeds(
            std::vector<SwerveModuleState> moduleStates,
            double attainableMaxSpeedMetersPerSecond)
        {
            double realMaxSpeed = moduleStates.at(0).speedMetersPerSecond;

            for (size_t i = 1; i < moduleStates.size(); i++)
            {
                realMaxSpeed = std::max(realMaxSpeed, moduleStates.at(i).speedMetersPerSecond);
            }

            if (realMaxSpeed > attainableMaxSpeedMetersPerSecond)
            {
                for (SwerveModuleState& moduleState : moduleStates)
                {
                    moduleState.speedMetersPerSecond = moduleState.speedMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
                }
            }
        }

        std::vector<team254_geometry::Translation2d> SwerveDriveKinematics::getModuleLocations() const
        {
            return m_modules;
        }

        int SwerveDriveKinematics::getNumModules() const
        {
            return m_numModules;
        }

    } // namespace team254_swerve
} // namespace ck
