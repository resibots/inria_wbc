
#include "trajectory_utils/CartesianTrj.h"
#include "trajectory_utils/trajectory_utils.h"

namespace trajectory_handler
{
    void computeTrj(std::shared_ptr<trajectory_utils::trajectory_generator> trjGenerator,
                    const KDL::Frame &start, const KDL::Frame &dest, double timeTrj,
                    bool reset = false)
    {
        if (reset)
            trjGenerator->resetTrajectory();
        else
            trjGenerator->addMinJerkTrj(start, dest, timeTrj);
        trjGenerator->resetInternalTime();
    }
}; // namespace trajectory_handler