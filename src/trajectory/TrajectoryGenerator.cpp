#include "ck_utilities/trajectory/TrajectoryGenerator.hpp"

namespace ck
{
    namespace trajectory
    {
        TrajectorySet::TrajectorySet(void)
        {
            // TODO: Fill with autos.
        }

        TrajectoryGenerator::TrajectoryGenerator(void)
        {
            this->mMotionPlanner = new planners::DriveMotionPlanner(); 
        }

        void TrajectoryGenerator::generateTrajectories(void)
        {
            if (this->mTrajectorySet == NULL)
            {
                printf("Generating trajectories...");
                mTrajectorySet = new TrajectorySet();
                printf("Trajectory generation completed!");
            }
        }

        TrajectorySet *TrajectoryGenerator::getTrajectorySet(void)
        {
            return this->mTrajectorySet;
        }
    } // namespace trajectory
} // namespace ck