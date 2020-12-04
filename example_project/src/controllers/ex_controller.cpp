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

#include "inria_wbc/controllers/ex_controller.hpp"
#include "inria_wbc/controllers/tasks.hpp"

using namespace tsid;
using namespace tsid::math;

namespace inria_wbc {
    namespace controllers {
        static Register<ExController> __ex_controller("ex-controller");

        ExController::ExController(const Params& params) : PosTracker(params)
        {
            parse_configuration_yaml(params.sot_config_path);
            if (verbose_)
                std::cout << "Talos pos tracker initialized" << std::endl;
        }
        void ExController::parse_configuration_yaml(const std::string& sot_config_path)
        {
            // you can open the file here and parse additional parameters

            // EX :
            // YAML::Node c = YAML::LoadFile(sot_config_path)["CONTROLLER"]["my_parameters"];
            // _my_parameter = c[my_parameter_name].as<my_parameter_type>();
            // if (verbose_) {
            //     std::cout << "_my_parameter:" << _my_parameter << std::endl;
            // }
        }

        void ExController::update(const SensorData& sensor_data)
        {
            //Do what you want with your parameters and solve
            //solve everything
            _solve();
        }
    } // namespace controllers
} // namespace inria_wbc
