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

// #include <tsid/tasks/task-joint-posVelAcc-bounds.hpp>

#include "inria_wbc/controllers/talos_pos_tracker.hpp"
#include "inria_wbc/controllers/tasks.hpp"
#include "inria_wbc/stabilizers/stabilizer.hpp"
#include "inria_wbc/utils/utils.hpp"

using namespace tsid;
using namespace tsid::math;

namespace inria_wbc {
    namespace controllers {
        static Register<TalosPosTracker> __talos_pos_tracking("talos-pos-tracker");

        TalosPosTracker::TalosPosTracker(const YAML::Node& config) : HumanoidPosTracker(config)
        {
            parse_torque_safety(config["CONTROLLER"]);

            //set the _torso_max_roll in the bounds for safety (for the stabilizer)
            auto names = robot_->model().names;
            names.erase(names.begin(), names.begin() + names.size() - robot_->na());
            auto q_lb = robot_->model().lowerPositionLimit.tail(robot_->na());
            auto q_ub = robot_->model().upperPositionLimit.tail(robot_->na());
            std::vector<std::string> to_limit = {"leg_left_2_joint", "leg_right_2_joint"};

            _torso_max_roll = IWBC_CHECK(config["CONTROLLER"]["torso_max_roll"].as<float>()) / 180 * M_PI;
            for (auto& n : to_limit) {
                IWBC_ASSERT(std::find(names.begin(), names.end(), n) != names.end(), "Talos should have ", n);
                auto id = std::distance(names.begin(), std::find(names.begin(), names.end(), n));

                IWBC_ASSERT((q_lb[id] <= q0_.tail(robot_->na()).transpose()[id]) && (q_ub[id] >= q0_.tail(robot_->na()).transpose()[id]), "Error in bounds, the torso limits are not viable");

                q_lb[id] = q0_.tail(robot_->na()).transpose()[id] - _torso_max_roll;
                q_ub[id] = q0_.tail(robot_->na()).transpose()[id] + _torso_max_roll;
            }

            bound_task()->setPositionBounds(q_lb, q_ub);
        }

        void TalosPosTracker::parse_torque_safety(const YAML::Node& config)
        {
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

                auto filtered_dof_names = this->all_dofs(true); // filter out mimics, include floating base (6dofs)
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

                if (verbose_) {
                    std::cout << "Collision detection:" << _use_torque_collision_detection << std::endl;
                    std::cout << "with thresholds" << std::endl;
                    for (size_t id = 0; id < _torque_collision_joints.size(); ++id)
                        std::cout << _torque_collision_joints[id] << ": " << _torque_collision_threshold(id) << std::endl;
                }
            }

            // parse compliance posture
            {
                auto c = IWBC_CHECK(config["compliance_posture"]);
                _use_compliance_posture = IWBC_CHECK(c["activated"].as<bool>());
                _compliance_posture_kp = IWBC_CHECK(c["kp"].as<double>());
                _compliance_posture_div = IWBC_CHECK(c["div"].as<double>());
                auto filter_window_size = IWBC_CHECK(c["filter_size"].as<int>());

                IWBC_ASSERT(_compliance_posture_div >= 0 && _compliance_posture_div <= 1, "compliance_posture div must be in [0,1].");
                IWBC_ASSERT(this->has_task("compliance_posture"), "talos_pos_tracker compliance_posture task is needed.");
                
                auto compliance_posture_task = this->task<tsid::tasks::TaskJointPosture>("compliance_posture");
                auto filtered_dof_names = this->all_dofs(true); // filter out mimics, !!! include floating base (6dofs)
                //for(auto& str : filtered_dof_names)
                //    std::cerr << "dof: " << str << std::endl;
                
                // check mask: create mask from torque collision joints index and execute logic-and with desired one
                Eigen::VectorXd valid_mask = Eigen::VectorXd::Zero(robot_->na());
                for(size_t id : _torque_collision_joints_ids)
                    valid_mask(id-6) = 1; // -6 to not consider floating base
                    
                Eigen::VectorXd compliance_mask = compliance_posture_task->getMask();
                Eigen::VectorXd valid_compliance_mask = compliance_mask.cwiseProduct(valid_mask);         

                if(compliance_mask != valid_compliance_mask)
                {
                    if(verbose_)
                        std::clog << "Non valid mask for compliance_posture task" << std::endl
                            << "Joints without torque sensors have been masked" << std::endl;
                    compliance_posture_task->setMask(valid_compliance_mask);
                }

                _joints_torque_filter = std::make_shared<estimators::MovingAverageFilter>(_torque_collision_joints.size(), filter_window_size);
                _joints_torque_filter->reset();

                if(verbose_)
                {
                    std::clog << "compliance posture: " << (_use_compliance_posture ? "true" : "false") << std::endl;
                    if(_use_compliance_posture)
                    {
                        std::clog << "\tkp: " << _compliance_posture_kp << std::endl;
                        std::clog << "\tdiv: " << _compliance_posture_div << std::endl;
                        std::clog << "\tfilter size: " << filter_window_size << std::endl;

                        for(int i=0; i < valid_compliance_mask.size(); ++i)
                            if(valid_compliance_mask(i) == 1)
                                std::clog << "compliance_posture acting on : " << filtered_dof_names[i+6] << std::endl;
                    }
                }        
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
            if(_use_compliance_posture)
            {
                IWBC_ASSERT(sensor_data.find("joints_torque") != sensor_data.end(), "compliance_posture task needs torque sensor measurements");

                Eigen::VectorXd actual_q = this->q(true);
                Eigen::VectorXd actual_tau = this->tau(true);
                
                Eigen::VectorXd compliance_mask = this->task<tsid::tasks::TaskJointPosture>("compliance_posture")->getMask();

                Eigen::VectorXd error = Eigen::VectorXd::Zero(actual_tau.size());
                const Eigen::VectorXd& joints_torque_raw = sensor_data.at("joints_torque");
                Eigen::VectorXd joints_torque = _joints_torque_filter->filter(joints_torque_raw);

                for(int i=0; i < _torque_collision_joints_ids.size(); ++i)
                {
                    auto idx = _torque_collision_joints_ids[i];
                    error(idx) = (joints_torque[i] - actual_tau(idx)) * compliance_mask(idx-6);
                }

                Eigen::VectorXd ref = actual_q - _compliance_posture_kp * error; // method 1
                //Eigen::VectorXd ref = actual_q + _compliance_posture_div * 10000 * (dt_*dt_) * (robot_->mass(tsid_->data()).inverse() * error); // method 2
                Eigen::VectorXd ref_no_fb = ref.tail(actual_q.size() - 6);
                for(int i=6; i < actual_q.size(); ++i)
                    if(this->task<tsid::tasks::TaskJointPosture>("compliance_posture")->getMask()(i-6) != 1)
                        ref_no_fb(i-6) = actual_q(i);
                
                // std::cerr << this->task<tsid::tasks::TaskJointPosture>("compliance_posture")->getMask().transpose() << std::endl;
                // std::cerr << (this->q().tail(this->q().size()-6) - ref_no_fb).transpose() << std::endl;
                this->task<tsid::tasks::TaskJointPosture>("compliance_posture")->setReference(trajs::to_sample(ref_no_fb));
            }
            
            if (_use_torque_collision_detection) {
                IWBC_ASSERT(sensor_data.find("joints_torque") != sensor_data.end(), "torque collision detection requires torque sensor data");
                IWBC_ASSERT(sensor_data.at("joints_torque").size() == _torque_collision_joints.size(), "torque sensor data has a wrong size. call torque_sensor_joints() for needed values");

                auto tsid_tau = utils::slice_vec(this->tau(), _torque_collision_joints_ids);
                _collision_detected = (false == _torque_collision_detection.check(tsid_tau, sensor_data.at("joints_torque")));
            }

            tsid::trajectories::TrajectorySample momentum_ref;
            if (_use_stabilizer && _stabilizer_configs[behavior_type_].use_momentum) {
                momentum_ref = get_full_momentum_ref();

                tsid::trajectories::TrajectorySample momentum_sample;
                _imu_angular_vel_filtered = _imu_angular_vel_filter->filter(sensor_data.at("imu_vel"));

                auto motion = robot()->frameVelocity(tsid()->data(), robot()->model().getFrameId("imu_link"));
                stabilizer::momentum_imu_admittance(dt_, _stabilizer_configs[behavior_type_].momentum_p, _stabilizer_configs[behavior_type_].momentum_d, motion.angular(), _imu_angular_vel_filtered, momentum_ref, momentum_sample);
                set_momentum_ref(momentum_sample);
            }

            HumanoidPosTracker::update(sensor_data);

            if (_use_stabilizer && _stabilizer_configs[behavior_type_].use_momentum)
                set_momentum_ref(momentum_ref);
        }

        void TalosPosTracker::clear_collision_detection()
        {
            _torque_collision_detection.reset();
            _torque_collision_filter->reset();
            _collision_detected = false;
        }

    } // namespace controllers
} // namespace inria_wbc
