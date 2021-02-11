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

        TalosPosTracker::TalosPosTracker(const Params& params) : PosTracker(params)
        {
            parse_configuration_yaml(params.sot_config_path);
            if (verbose_)
                std::cout << "Talos pos tracker initialized" << std::endl;
        }

        void TalosPosTracker::parse_configuration_yaml(const std::string& sot_config_path)
        {
            // init stabilizer
            {
                YAML::Node c = YAML::LoadFile(sot_config_path)["CONTROLLER"]["stabilizer"];
                _use_stabilizer = c["activated"].as<bool>();
                _stabilizer_p.resize(6);
                _stabilizer_d.resize(6);
                _stabilizer_p.setZero();
                _stabilizer_d.setZero();
                IWBC_ASSERT(c["p"].as<std::vector<double>>().size() == 6, "you need 6 coefficient in p for the stabilizer");
                IWBC_ASSERT(c["d"].as<std::vector<double>>().size() == 6, "you need 6 coefficient in d for the stabilizer");
                _stabilizer_p = Eigen::VectorXd::Map(c["p"].as<std::vector<double>>().data(), c["p"].as<std::vector<double>>().size());
                _stabilizer_d = Eigen::VectorXd::Map(c["d"].as<std::vector<double>>().data(), c["d"].as<std::vector<double>>().size());
                _stabilizer_p_ankle = Eigen::Vector2d(c["p_ankle"].as<std::vector<double>>().data());
                auto history = c["filter_size"].as<int>();
                _cop_estimator.set_history_size(history);
            }

            // init collision detection
            {
                YAML::Node c = YAML::LoadFile(sot_config_path)["CONTROLLER"]["collision_detection"];
                _use_torque_collision_detection = c["activated"].as<bool>();
                auto filter_window_size = c["filter_size"].as<int>();
                auto max_invalid = c["max_invalid"].as<int>();

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
                    auto path = boost::filesystem::path(sot_config_path).parent_path();
                    auto p_thresh = path / boost::filesystem::path(c["thresholds"].as<std::string>());
                    parse_collision_thresholds(p_thresh.string());
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
            YAML::Node config = YAML::LoadFile(config_path);
            for (size_t jid = 0; jid < _torque_collision_joints.size(); ++jid) {
                std::string joint = _torque_collision_joints[jid];
                if (config[joint])
                    _torque_collision_threshold(jid) = config[joint].as<double>();
            }

            return;
        }

        void TalosPosTracker::update(const SensorData& sensor_data)
        {

            auto com_ref = com_task()->getReference();
            auto cl = activated_contacts();
            for (auto& contact_name : cl) {
                auto pos = contact(contact_name)->getMotionTask().getReference().pos;
                pinocchio::SE3 se3;
                tsid::math::vectorToSE3(pos, se3);
                _contact_ref[contact_name] = se3;
            }
            auto left_ankle_ref = get_se3_ref("lf");
            auto right_ankle_ref = get_se3_ref("rf");

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
                    Eigen::Vector2d cor = ref.head(2) - _cop_estimator.cop_filtered();

                    // [not classic] we correct by the velocity of the CoM instead of the CoP because we have an IMU for this
                    Eigen::Vector2d cor_v = sensor_data.at("velocity").block<2, 1>(0, 0);

                    Eigen::Vector2d total_corr = _stabilizer_p.block(0, 0, 1, 2).array() * cor.array() + _stabilizer_d.block(0, 0, 1, 2).array() * cor_v.array();
                    Eigen::VectorXd ref_m = com_ref.pos - Eigen::Vector3d(total_corr(0), total_corr(1), 0);
                    total_corr = _stabilizer_p.block(2, 0, 1, 2).array() * cor.array() + _stabilizer_d.block(2, 0, 1, 2).array() * cor_v.array();
                    Eigen::VectorXd vref_m = com_ref.vel - (Eigen::Vector3d(total_corr(0), total_corr(1), 0) / dt_);
                    total_corr = _stabilizer_p.block(4, 0, 1, 2).array() * cor.array() + _stabilizer_d.block(4, 0, 1, 2).array() * cor_v.array();
                    Eigen::VectorXd aref_m = com_ref.acc - (Eigen::Vector3d(total_corr(0), total_corr(1), 0) / (dt_ * dt_));
                    tsid::trajectories::TrajectorySample sample;
                    sample.pos = ref_m;
                    sample.vel = vref_m;
                    sample.acc = aref_m;
                    set_com_ref(sample);
                    // set_com_ref(ref_m);
                }

                if (cop_ok && !std::isnan(_cop_estimator.lcop_filtered()(0)) && !std::isnan(_cop_estimator.lcop_filtered()(1)) && std::find(cl.begin(), cl.end(), "contact_lfoot") != cl.end()) {
                    cop_admittance(_stabilizer_p_ankle, _cop_estimator.lcop_filtered(), "l", _contact_ref, left_ankle_ref);
                }

                if (cop_ok && !std::isnan(_cop_estimator.rcop_filtered()(0)) && !std::isnan(_cop_estimator.rcop_filtered()(1)) && std::find(cl.begin(), cl.end(), "contact_rfoot") != cl.end()) {
                    cop_admittance(_stabilizer_p_ankle, _cop_estimator.rcop_filtered(), "r", _contact_ref, right_ankle_ref);
                }
            }

            if (_use_torque_collision_detection) {
                IWBC_ASSERT(sensor_data.find("joints_torque") != sensor_data.end(), "torque collision detection requires torque sensor data");
                IWBC_ASSERT(sensor_data.at("joints_torque").size() == _torque_collision_joints.size(), "torque sensor data has a wrong size. call torque_sensor_joints() for needed values");

                auto tsid_tau = utils::slice_vec(this->tau(), _torque_collision_joints_ids);
                _collision_detected = (false == _torque_collision_detection.check(tsid_tau, sensor_data.at("joints_torque")));
            }

            // solve everything
            _solve();

            // set the CoM back (useful if the behavior does not the set the ref at each timestep)
            set_com_ref(com_ref);
            set_se3_ref(left_ankle_ref, "lf");
            set_se3_ref(right_ankle_ref, "lf");
            for (auto& contact_name : cl) {
                contact(contact_name)->setReference(_contact_ref[contact_name]);
            }
        }

        void TalosPosTracker::cop_admittance(Eigen::Vector2d pd_gains, Eigen::Vector2d cop_foot, std::string foot, std::map<std::string, pinocchio::SE3> contact_ref, pinocchio::SE3 ankle_ref)
        {
            int sign = 1;
            if (foot == "l")
                sign = -1;

            double roll = pd_gains[1] * cop_foot(1);
            double pitch = -pd_gains[0] * cop_foot(0);

            auto euler = ankle_ref.rotation().eulerAngles(0, 1, 2);
            if (foot == "l")
                std::cout << cop_foot(0) << " " << cop_foot(1) << std::endl;
            auto q = Eigen::AngleAxisd(euler[0] + roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(euler[1] + pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
            ankle_ref.rotation() = q.toRotationMatrix();
            set_se3_ref(ankle_ref, foot + "f");

            contact_ref["contact_" + foot + "foot"].rotation() = q.toRotationMatrix();
            contact("contact_" + foot + "foot")->setReference(contact_ref["contact_" + foot + "foot"]);
        }

        void TalosPosTracker::clear_collision_detection()
        {
            _torque_collision_detection.reset();
            _torque_collision_filter->reset();
            _collision_detected = false;
        }

    } // namespace controllers
} // namespace inria_wbc
