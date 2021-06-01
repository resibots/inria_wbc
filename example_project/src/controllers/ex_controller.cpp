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

        ExController::ExController(const YAML::Node& config) : PosTracker(config)
        {
            parse_configuration(config["CONTROLLER"]);
            if (verbose_)
                std::cout << "Talos pos tracker initialized" << std::endl;
        }
        void ExController::parse_configuration(const YAML::Node& config)
        {
            // you can open the file here and parse additional parameters

            // EX :
            // YAML::Node c = YAML::LoadFile(sot_config_path)["CONTROLLER"]["my_parameters"];
            // _my_parameter = c[my_parameter_name].as<my_parameter_type>();
            // if (verbose_) {
            //     std::cout << "_my_parameter:" << _my_parameter << std::endl;
            // }
            _closed_loop = IWBC_CHECK(config["closed_loop"].as<bool>());
        }

        void ExController::update(const SensorData& sensor_data)
        {
            //Do what you want with your parameters and solve
            //solve everything

           if (_closed_loop) {
                IWBC_ASSERT(sensor_data.find("floating_base_position") != sensor_data.end(),
                    "we need the floating base position in closed loop mode!");
                IWBC_ASSERT(sensor_data.find("floating_base_velocity") != sensor_data.end(),
                    "we need the floating base velocity in closed loop mode!");
                IWBC_ASSERT(sensor_data.find("positions") != sensor_data.end(),
                    "we need the joint positions in closed loop mode!");
                IWBC_ASSERT(sensor_data.find("joint_velocities") != sensor_data.end(),
                    "we need the joint velocities in closed loop mode!");

                Eigen::VectorXd q_tsid(q_tsid_.size()), dq(v_tsid_.size());
                auto pos = sensor_data.at("positions");
                auto vel = sensor_data.at("joint_velocities");
                auto fb_pos = sensor_data.at("floating_base_position");
                auto fb_vel = sensor_data.at("floating_base_velocity");

                IWBC_ASSERT(vel.size() + fb_vel.size() == v_tsid_.size(),
                    "Joint velocities do not have the correct size:", vel.size() + fb_vel.size(), " vs (expected)", v_tsid_.size());
                IWBC_ASSERT(pos.size() + fb_pos.size() == q_tsid_.size(),
                    "Joint positions do not have the correct size:", pos.size() + fb_pos.size(), " vs (expected)", q_tsid_.size());

                q_tsid << fb_pos, pos;
                dq << fb_vel, vel;

                _solve(q_tsid, dq);
            } else {
                _solve();
            }
        }
    } // namespace controllers
} // namespace inria_wbc
