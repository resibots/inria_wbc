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

#include <boost/filesystem.hpp>
#include <tsid/solvers/solver-HQP-base.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/utils.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/utils/stop-watch.hpp>

#include "inria_wbc/controllers/talos_pos_tracker.hpp"
#include "inria_wbc/controllers/tasks.hpp"
#include "inria_wbc/stabilizers/stabilizer.hpp"
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
                YAML::Node c = IWBC_CHECK(YAML::LoadFile(sot_config_path)["CONTROLLER"]["stabilizer"]);
                _use_stabilizer = IWBC_CHECK(c["activated"].as<bool>());
                _torso_max_roll = IWBC_CHECK(c["torso_max_roll"].as<double>());

                _stabilizer_p.resize(6);
                _stabilizer_d.resize(6);
                _stabilizer_p_ankle.resize(6);
                _stabilizer_p_ffda.resize(3);

                _stabilizer_p.setZero();
                _stabilizer_d.setZero();
                _stabilizer_p_ankle.setZero();
                _stabilizer_p_ffda.setZero();

                IWBC_ASSERT(IWBC_CHECK(c["p"].as<std::vector<double>>()).size() == 6, "you need 6 coefficient in p for the stabilizer");
                IWBC_ASSERT(IWBC_CHECK(c["d"].as<std::vector<double>>()).size() == 6, "you need 6 coefficient in d for the stabilizer");
                IWBC_ASSERT(IWBC_CHECK(c["p_ankle"].as<std::vector<double>>()).size() == 6, "you need 6 coefficient in p_ankle for the stabilizer");
                IWBC_ASSERT(IWBC_CHECK(c["p_ffda"].as<std::vector<double>>()).size() == 3, "you need 3 coefficient in p_ffda for the stabilizer");

                _stabilizer_p = Eigen::VectorXd::Map(IWBC_CHECK(c["p"].as<std::vector<double>>()).data(), _stabilizer_p.size());
                _stabilizer_d = Eigen::VectorXd::Map(IWBC_CHECK(c["d"].as<std::vector<double>>()).data(), _stabilizer_d.size());
                _stabilizer_p_ankle = Eigen::VectorXd::Map(IWBC_CHECK(c["p_ankle"].as<std::vector<double>>()).data(), _stabilizer_p_ankle.size());
                _stabilizer_p_ffda = Eigen::VectorXd::Map(IWBC_CHECK(c["p_ffda"].as<std::vector<double>>()).data(), _stabilizer_p_ffda.size());

                auto history = c["filter_size"].as<int>();
                _cop_estimator.set_history_size(history);

                _lf_force_filtered.setZero();
                _rf_force_filtered.setZero();
                _lf_force_filter = std::make_shared<estimators::MovingAverageFilter>(3, history); //force data of size 3
                _rf_force_filter = std::make_shared<estimators::MovingAverageFilter>(3, history); //force data of size 3
            }

            // init collision detection
            {
                YAML::Node c = IWBC_CHECK(YAML::LoadFile(sot_config_path)["CONTROLLER"]["collision_detection"]);
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
                    auto path = boost::filesystem::path(sot_config_path).parent_path();
                    auto p_thresh = IWBC_CHECK(path / boost::filesystem::path(c["thresholds"].as<std::string>()));
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
                std::cout << "P ANKLE:" << _stabilizer_p_ankle.transpose() << std::endl;
                std::cout << "P FFDA:" << _stabilizer_p_ffda.transpose() << std::endl;

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

            auto ac = activated_contacts_;
            for (auto& contact_name : ac) {
                auto pos = contact(contact_name)->getMotionTask().getReference().pos;
                pinocchio::SE3 se3;
                tsid::math::vectorToSE3(pos, se3);
                _contact_ref[contact_name] = se3;
            }
            auto com_ref = com_task()->getReference();
            auto left_ankle_ref = get_full_se3_ref("lf");
            auto right_ankle_ref = get_full_se3_ref("rf");
            auto torso_ref = get_full_se3_ref("torso");

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

                tsid::trajectories::TrajectorySample se3_sample, contact_sample, com_sample, torso_sample;
                // modify the CoM reference (stabilizer) if the CoP is valid
                if (cop_ok
                    && !std::isnan(_cop_estimator.cop_filtered()(0))
                    && !std::isnan(_cop_estimator.cop_filtered()(1))) {

                    stabilizer::com_admittance(dt_, _stabilizer_p, _stabilizer_d, sensor_data.at("velocity"), _cop_estimator.cop_filtered(), com_ref, tsid_->data(), com_sample);
                    set_com_ref(com_sample);
                }

                if (cop_ok
                    && !std::isnan(_cop_estimator.lcop_filtered()(0))
                    && !std::isnan(_cop_estimator.lcop_filtered()(1))
                    && std::find(ac.begin(), ac.end(), "contact_lfoot") != ac.end()) {

                    stabilizer::ankle_admittance(dt_, "l", _cop_estimator.lcop_filtered(), _stabilizer_p_ankle, get_se3_ref("lf"), _contact_ref, contact_sample, se3_sample);
                    set_se3_ref(se3_sample, "lf");
                    contact("contact_lfoot")->setReference(contact_sample);
                }

                if (cop_ok
                    && !std::isnan(_cop_estimator.rcop_filtered()(0))
                    && !std::isnan(_cop_estimator.rcop_filtered()(1))
                    && std::find(ac.begin(), ac.end(), "contact_rfoot") != ac.end()) {

                    stabilizer::ankle_admittance(dt_, "r", _cop_estimator.rcop_filtered(), _stabilizer_p_ankle, get_se3_ref("rf"), _contact_ref, contact_sample, se3_sample);
                    set_se3_ref(se3_sample, "rf");
                    contact("contact_rfoot")->setReference(contact_sample);
                }

                // if the foot is on the ground
                if (sensor_data.at("lf_force").norm() > _cop_estimator.fmin())
                    _lf_force_filtered = _lf_force_filter->filter(sensor_data.at("lf_force"));

                if (sensor_data.at("rf_force").norm() > _cop_estimator.fmin())
                    _rf_force_filtered = _rf_force_filter->filter(sensor_data.at("rf_force"));

                if (activated_contacts_forces_.find("contact_rfoot") != activated_contacts_forces_.end()
                    && activated_contacts_forces_.find("contact_lfoot") != activated_contacts_forces_.end()
                    && _lf_force_filter->data_ready()
                    && _rf_force_filter->data_ready()) {

                    //normal force of the contacts from tsid
                    double lf_normal_force = contact("contact_lfoot")->Contact6d::getNormalForce(activated_contacts_forces_["contact_lfoot"]);
                    double rf_normal_force = contact("contact_rfoot")->Contact6d::getNormalForce(activated_contacts_forces_["contact_rfoot"]);

                    stabilizer::foot_force_difference_admittance(dt_, _torso_max_roll, _stabilizer_p_ffda, get_se3_ref("torso"), lf_normal_force, rf_normal_force, _lf_force_filtered, _rf_force_filtered, torso_sample);
                    set_se3_ref(torso_sample, "torso");
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
            if (_use_stabilizer) {
                set_com_ref(com_ref);
                set_se3_ref(left_ankle_ref, "lf");
                set_se3_ref(right_ankle_ref, "rf");
                set_se3_ref(torso_ref, "torso");

                for (auto& contact_name : ac) {
                    contact(contact_name)->Contact6d::setReference(_contact_ref[contact_name]);
                }
            }
        }

        void TalosPosTracker::clear_collision_detection()
        {
            _torque_collision_detection.reset();
            _torque_collision_filter->reset();
            _collision_detected = false;
        }

    } // namespace controllers
} // namespace inria_wbc
