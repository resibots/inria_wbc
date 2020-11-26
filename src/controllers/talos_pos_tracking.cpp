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

#include "inria_wbc/controllers/talos_pos_tracking.hpp"
#include "inria_wbc/controllers/tasks.hpp"

using namespace tsid;
using namespace tsid::math;

namespace inria_wbc {
    namespace controllers {
        static Register<TalosPosTracking> __talos_pos_tracking("talos-pos-tracking");

        TalosPosTracking::TalosPosTracking(const Params& params) : PosTracker(params)
        {
            parse_configuration_yaml(params.sot_config_path);
            if (verbose_)
                std::cout << "Talos pos tracker initialized" << std::endl;
        }

        void TalosPosTracking::parse_configuration_yaml(const std::string& sot_config_path)
        {
            YAML::Node config = YAML::LoadFile(sot_config_path);
            _use_stabilizer = config["CONTROLLER"]["stabilizer"]["activated"].as<bool>();
            _stabilizer_p = Eigen::Vector2d(config["CONTROLLER"]["stabilizer"]["p"].as<std::vector<double>>().data());
            _stabilizer_d = Eigen::Vector2d(config["CONTROLLER"]["stabilizer"]["d"].as<std::vector<double>>().data());
            auto history = config["CONTROLLER"]["stabilizer"]["filter_size"].as<int>();
            _cop_estimator.set_history_size(history);

            if (verbose_) {
                std::cout << "Stabilizer:" << _use_stabilizer << std::endl;
                std::cout << "P:" << _stabilizer_p.transpose() << std::endl;
                std::cout << "D:" << _stabilizer_d.transpose() << std::endl;
            }
        }

        void TalosPosTracking::update(const SensorData& sensor_data)
        {
            auto com_ref = com_task()->getReference().pos;

            IWBC_ASSERT(sensor_data.find("lf_torque") != sensor_data.end(), "the stabilizer needs the LF torque");
            IWBC_ASSERT(sensor_data.find("rf_torque") != sensor_data.end(), "the stabilizer needs the need RF torque");
            IWBC_ASSERT(sensor_data.find("velocity") != sensor_data.end(), "the stabilizer needs the need velocity");

            // estimate the CoP / ZMP
            bool cop_ok = _cop_estimator.update(com_ref.head(2),
                model_joint_pos("leg_left_6_joint").translation(),
                model_joint_pos("leg_right_6_joint").translation(),
                sensor_data.at("lf_torque"), sensor_data.at("lf_force"),
                sensor_data.at("rf_torque"), sensor_data.at("rf_force"));
            // modify the CoM reference (stabilizer) if the CoP is valid
            if (_use_stabilizer && cop_ok && !std::isnan(_cop_estimator.cop_filtered()(0)) && !std::isnan(_cop_estimator.cop_filtered()(1))) {
                // the expected zmp given CoM in x is x - z_c / g \ddot{x} (LIPM equations)
                // CoM = CoP+zc/g \ddot{x}
                // see Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point
                // see eq.24 of Biped Walking Stabilization Based on Linear Inverted Pendulum Tracking
                // see eq. 21 of Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control
                Eigen::Vector2d a = tsid_->data().acom[0].head<2>();
                Eigen::Vector3d com = tsid_->data().com[0];
                Eigen::Vector2d ref = com.head<2>() - com(2) / 9.81 * a; //com because this is the target
                auto cop = _cop_estimator.cop_filtered();
                Eigen::Vector2d cor = _stabilizer_p.array() * (ref.head(2) - _cop_estimator.cop_filtered()).array();

                // [not classic] we correct by the velocity of the CoM instead of the CoP because we have an IMU for this
                Eigen::Vector2d cor_v = _stabilizer_d.array() * sensor_data.at("velocity").block<2,1>(0,0).array();
                cor += cor_v;

                Eigen::VectorXd ref_m = com_ref - Eigen::Vector3d(cor(0), cor(1), 0);
                set_com_ref(ref_m);
            }

            // solve everything
            _solve();

            // set the CoM back (useful if the behavior does not the set the ref at each timestep)
            set_com_ref(com_ref);
        }

    } // namespace controllers
} // namespace inria_wbc
