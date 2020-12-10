#include <Eigen/Core>
#include <iomanip>
#include <map>
#include <memory>
#include <utility>
#include <vector>

/* Pinocchio !!!! NEED TO BE INCLUDED BEFORE BOOST*/
#include <pinocchio/algorithm/joint-configuration.hpp> // integrate
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <tsid/solvers/solver-HQP-base.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/utils.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/utils/stop-watch.hpp>

#include <boost/filesystem.hpp>

#include "inria_wbc/controllers/franka_pos_tracker.hpp"
#include "inria_wbc/controllers/tasks.hpp"

using namespace tsid;
using namespace tsid::math;

namespace inria_wbc {
    namespace controllers {
        static Register<FrankaPosTracker> __franka_pos_tracker("franka-pos-tracker");

        FrankaPosTracker::FrankaPosTracker(const Params& params) : PosTracker(params)
        {
            if (verbose_)
                std::cout << "Franka pos tracker initialized" << std::endl;
        }

        void FrankaPosTracker::update(const SensorData& sensor_data)
        {
            _solve();
        }
    } // namespace controllers
} // namespace inria_wbc
