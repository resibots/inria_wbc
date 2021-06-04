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

#include "inria_wbc/controllers/talos_pos_tracker.hpp"
#include "inria_wbc/controllers/tasks.hpp"
#include "inria_wbc/utils/utils.hpp"

using namespace tsid;
using namespace tsid::math;

namespace inria_wbc {
    namespace controllers {
        static Register<TalosPosTracker> __talos_pos_tracking("talos-pos-tracker");

        TalosPosTracker::TalosPosTracker(const YAML::Node& config) : PosTracker(config)
        {
            parse_configuration(config["CONTROLLER"]);
            if (verbose_)
                std::cout << "Talos pos tracker initialized" << std::endl;
        }

        void TalosPosTracker::parse_configuration(const YAML::Node& config)
        {
            // closed loop
            _closed_loop = IWBC_CHECK(config["closed_loop"].as<bool>());

            // init stabilizer
            {
                auto c = IWBC_CHECK(config["stabilizer"]);
                _use_stabilizer = IWBC_CHECK(c["activated"].as<bool>());
                _stabilizer_p = IWBC_CHECK(Eigen::Vector2d(c["p"].as<std::vector<double>>().data()));
                _stabilizer_d = IWBC_CHECK(Eigen::Vector2d(c["d"].as<std::vector<double>>().data()));
                auto history = IWBC_CHECK(c["filter_size"].as<int>());
                _cop_estimator.set_history_size(history);
            }

            // init collision detection
            {
                auto c = IWBC_CHECK(config["collision_detection"]);
                _use_torque_collision_detection = IWBC_CHECK(c["activated"].as<bool>());
                auto filter_window_size = IWBC_CHECK(c["filter_size"].as<int>());
                auto max_invalid = IWBC_CHECK(c["max_invalid"].as<int>());

                _torque_collision_joints = {
                    "leg_left_1_joint", "leg_left_2_joint", "leg_left_3_joint", "leg_left_4_joint", "leg_left_5_joint", "leg_left_6_joint",
                    "leg_right_1_joint", "leg_right_2_joint", "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint", "leg_right_6_joint",
                    "torso_1_joint", "torso_2_joint",
                    "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint",
                    "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint"};

                auto filtered_dof_names = this->all_dofs(true); // filter out mimics
                for (const auto& joint : _torque_collision_joints) {
                    auto it = std::find(filtered_dof_names.begin(), filtered_dof_names.end(), joint);
                    _torque_collision_joints_ids.push_back(std::distance(filtered_dof_names.begin(), it));
                }

                _torque_collision_threshold.resize(_torque_collision_joints.size());
                _torque_collision_threshold << 3.5e+05, 3.9e+05, 2.9e+05, 4.4e+05, 5.7e+05, 2.4e+05,
                    3.5e+05, 3.9e+05, 2.9e+05, 4.4e+05, 5.7e+05, 2.4e+05,
                    1e+01, 1e+01,
                    1e+01, 1e+01, 1e+01, 1e+01,
                    1e+01, 1e+01, 1e+01, 1e+01;

                // update thresholds from file (if any)
                if (c["thresholds"]) {
                    auto path = IWBC_CHECK(config["base_path"].as<std::string>());
                    auto p_thresh = IWBC_CHECK(path + "/" + c["thresholds"].as<std::string>());
                    parse_collision_thresholds(p_thresh);
                }

                _torque_collision_filter = std::make_shared<estimators::MovingAverageFilter>(_torque_collision_joints.size(), filter_window_size);

                _torque_collision_detection = safety::TorqueCollisionDetection(_torque_collision_threshold);
                _torque_collision_detection.set_max_consecutive_invalid(max_invalid);
                _torque_collision_detection.set_filter(_torque_collision_filter);
            }

            if (verbose_) {
                std::cout << "Stabilizer:" << _use_stabilizer << std::endl;
                std::cout << "P:" << _stabilizer_p.transpose() << std::endl;
                std::cout << "D:" << _stabilizer_d.transpose() << std::endl;

                std::cout << "Collision detection:" << _use_torque_collision_detection << std::endl;
                std::cout << "with thresholds" << std::endl;
                for (size_t id = 0; id < _torque_collision_joints.size(); ++id)
                    std::cout << _torque_collision_joints[id] << ": " << _torque_collision_threshold(id) << std::endl;
            }
        }

        void TalosPosTracker::parse_collision_thresholds(const std::string& config_path)
        {
            YAML::Node config = IWBC_CHECK(YAML::LoadFile(config_path));
            for (size_t jid = 0; jid < _torque_collision_joints.size(); ++jid) {
                std::string joint = _torque_collision_joints[jid];
                if (config[joint])
                    _torque_collision_threshold(jid) = IWBC_CHECK(config[joint].as<double>());
            }

            return;
        }

        void TalosPosTracker::update(const SensorData& sensor_data)
        {

            auto com_ref = com_task()->getReference();

            if (_use_stabilizer) {
                IWBC_ASSERT(sensor_data.find("lf_torque") != sensor_data.end(), "the stabilizer needs the LF torque");
                IWBC_ASSERT(sensor_data.find("rf_torque") != sensor_data.end(), "the stabilizer needs the RF torque");
                IWBC_ASSERT(sensor_data.find("velocity") != sensor_data.end(), "the stabilizer needs the velocity");

                // estimate the CoP / ZMP
                bool cop_ok = _cop_estimator.update(com_ref.pos.head(2),
                    model_joint_pos("leg_left_6_joint").translation(),
                    model_joint_pos("leg_right_6_joint").translation(),
                    sensor_data.at("lf_torque"), sensor_data.at("lf_force"),
                    sensor_data.at("rf_torque"), sensor_data.at("rf_force"));

                // modify the CoM reference (stabilizer) if the CoP is valid
                if (cop_ok && !std::isnan(_cop_estimator.cop_filtered()(0)) && !std::isnan(_cop_estimator.cop_filtered()(1))) {
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
                    Eigen::Vector2d cor_v = _stabilizer_d.array() * sensor_data.at("velocity").block<2, 1>(0, 0).array();
                    cor += cor_v;

                    Eigen::VectorXd ref_m = com_ref.pos - Eigen::Vector3d(cor(0), cor(1), 0);
                    Eigen::VectorXd vref_m = com_ref.vel - (Eigen::Vector3d(cor(0), cor(1), 0) / dt_);
                    Eigen::VectorXd aref_m = com_ref.acc - (Eigen::Vector3d(cor(0), cor(1), 0) / (dt_ * dt_));
                    tsid::trajectories::TrajectorySample sample;
                    sample.pos = ref_m;
                    sample.vel = vref_m;
                    sample.acc.setZero(ref_m.size());
                    set_com_ref(sample);
                    // set_com_ref(ref_m);
                }
            }

            if (_use_torque_collision_detection) {
                IWBC_ASSERT(sensor_data.find("joints_torque") != sensor_data.end(), "torque collision detection requires torque sensor data");
                IWBC_ASSERT(sensor_data.at("joints_torque").size() == _torque_collision_joints.size(), "torque sensor data has a wrong size. call torque_sensor_joints() for needed values");

                auto tsid_tau = utils::slice_vec(this->tau(), _torque_collision_joints_ids);
                _collision_detected = (false == _torque_collision_detection.check(tsid_tau, sensor_data.at("joints_torque")));
            }

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
            }
            else {
                _solve();
            }

            // set the CoM back (useful if the behavior does not the set the ref at each timestep)
            set_com_ref(com_ref);
        }

        void TalosPosTracker::clear_collision_detection()
        {
            _torque_collision_detection.reset();
            _torque_collision_filter->reset();
            _collision_detected = false;
        }

    } // namespace controllers
} // namespace inria_wbc
