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
        static Register<HumanoidPosTracker> __humanoid_pos_tracker("humanoid-pos-tracker");

        HumanoidPosTracker::HumanoidPosTracker(const YAML::Node& config) : PosTracker(config)
        {
            behavior_type_ = behavior_types::FIXED_BASE;
            auto c = IWBC_CHECK(config["CONTROLLER"]);
            parse_stabilizer(c);
            if (verbose_)
                std::cout << "Humanoid pos tracker initialized" << std::endl;
            init_com();
        }

        // this is called by the constructor
        // we move the CoM to the center of the feet, to always start in the most stable configuration
        // (this should already be the case with a propre SRDF)
        void HumanoidPosTracker::init_com()
        {
            if (tasks_.find("com") != tasks_.end()) {
                auto com_init = this->com();
                auto com_final = this->com();

                // check that we have one task for each foot and that they are in contact
                if (tasks_.find("lf") != tasks_.end() && tasks_.find("rf") != tasks_.end() && contacts_.find("contact_lfoot") != contacts_.end() && contacts_.find("contact_rfoot") != contacts_.end()) {
                    com_final.head(2) = (this->get_se3_ref("lf").translation().head(2) + this->get_se3_ref("rf").translation().head(2)) / 2;
                    if ((this->com() - com_final).norm() > 0.01) // 1 cm
                        IWBC_ERROR("Wrong starting configuration: the CoM needs to be between the two feet with less than 1cm difference, but the distance is ", (this->com() - com_final).norm());

                    this->set_com_ref(com_final);
                    if (verbose_)
                        std::cout << "Taking initial com reference in the middle of the support polygon" << std::endl;
                }
                else {
                    IWBC_ERROR("init_com: contact_rfoot or contact_lfoot or lf or rf is missing, cannot compute initial com reference");
                }

                if (verbose_) {
                    std::cout << "Previous com ref: " << com_init.transpose() << std::endl;
                    std::cout << "New com ref: " << com_final.transpose() << std::endl;
                }
                this->set_com_ref(com_final);
            }
            else {
                IWBC_ERROR("init_com: no com task");
            }
        }

        void HumanoidPosTracker::parse_stabilizer(const YAML::Node& config)
        {
            auto c = IWBC_CHECK(config["stabilizer"]);
            _use_stabilizer = IWBC_CHECK(c["activated"].as<bool>());

            auto path = IWBC_CHECK(config["base_path"].as<std::string>());
            _stabilizer_configs[behavior_types::FIXED_BASE] = inria_wbc::stabilizer::parse_stab_conf(IWBC_CHECK(path + "/" + c["params_fixed_base"].as<std::string>()));
            _stabilizer_configs[behavior_types::SINGLE_SUPPORT] = inria_wbc::stabilizer::parse_stab_conf(IWBC_CHECK(path + "/" + c["params_ss"].as<std::string>()));
            _stabilizer_configs[behavior_types::DOUBLE_SUPPORT] = inria_wbc::stabilizer::parse_stab_conf(IWBC_CHECK(path + "/" + c["params_ds"].as<std::string>()));

            if (verbose_) {
                for (auto& c : _stabilizer_configs) {
                    std::cout << c.first << std::endl;
                    std::cout << c.second << std::endl;
                }
                std::cout << "using " << behavior_type_ << std::endl;
            }

            auto history = _stabilizer_configs[behavior_type_].filter_size;
            _cop_estimator.set_history_size(history);

            _lf_force_filtered.setZero();
            _rf_force_filtered.setZero();
            _lf_force_filter = std::make_shared<estimators::MovingAverageFilter>(3, history); //force data of size 3
            _rf_force_filter = std::make_shared<estimators::MovingAverageFilter>(3, history); //force data of size 3

            _lf_torque_filtered.setZero();
            _rf_torque_filtered.setZero();
            _lf_torque_filter = std::make_shared<estimators::MovingAverageFilter>(3, history); //force data of size 3
            _rf_torque_filter = std::make_shared<estimators::MovingAverageFilter>(3, history); //force data of size 3

            _imu_angular_vel_filtered.setZero();
            _imu_angular_vel_filter = std::make_shared<estimators::MovingAverageFilter>(3, history); //angular vel data of size 3
        }

        void HumanoidPosTracker::set_behavior_type(const std::string& bt)
        {
            Controller::set_behavior_type(bt);

            if (_stabilizer_configs.find(behavior_type_) != _stabilizer_configs.end()) {
                auto history = _stabilizer_configs[behavior_type_].filter_size;
                _cop_estimator.set_history_size(history);
                _lf_force_filter->set_window_size(history);
                _rf_force_filter->set_window_size(history);
                _lf_torque_filter->set_window_size(history);
                _rf_torque_filter->set_window_size(history);
                _imu_angular_vel_filter->set_window_size(history);
            }
            else {
                IWBC_ERROR("_stabilizer_configs does not have ", behavior_type_);
            }
        }

        void HumanoidPosTracker::update(const SensorData& sensor_data)
        {
            if (_stabilizer_configs.find(behavior_type_) == _stabilizer_configs.end())
                IWBC_ERROR("_stabilizer_configs does not have ", behavior_type_);

            std::map<std::string, tsid::trajectories::TrajectorySample> contact_sample_ref;
            std::map<std::string, pinocchio::SE3> contact_se3_ref;
            std::map<std::string, Eigen::Matrix<double, 6, 1>> contact_force_ref;

            auto ac = activated_contacts_;
            for (auto& contact_name : ac) {
                pinocchio::SE3 se3;
                auto contact_pos = contact(contact_name)->getMotionTask().getReference().getValue();
                tsid::math::vectorToSE3(contact_pos, se3);
                contact_se3_ref[contact_name] = se3;
                contact_sample_ref[contact_name] = contact(contact_name)->getMotionTask().getReference();
                contact_force_ref[contact_name] = contact(contact_name)->getForceReference();
            }

            auto com_ref = com_task()->getReference();
            auto left_ankle_ref = get_full_se3_ref("lf");
            auto right_ankle_ref = get_full_se3_ref("rf");
            auto torso_ref = get_full_se3_ref("torso");

            if (_use_stabilizer) {
                IWBC_ASSERT(sensor_data.find("lf_torque") != sensor_data.end(), "the stabilizer needs the LF torque");
                IWBC_ASSERT(sensor_data.find("rf_torque") != sensor_data.end(), "the stabilizer needs the RF torque");
                IWBC_ASSERT(sensor_data.find("velocity") != sensor_data.end(), "the stabilizer needs the velocity");
                IWBC_ASSERT(sensor_data.find("imu_vel") != sensor_data.end(), "the stabilizer imu needs angular velocity");

                // we retrieve the tracked frame from the contact task as the frames have different names in different robots
                // the ankle = where is the f/t sensor
                auto left_ankle_name = robot_->model().frames[contact("contact_lfoot")->getMotionTask().frame_id()].name;
                auto right_ankle_name = robot_->model().frames[contact("contact_rfoot")->getMotionTask().frame_id()].name;

                // estimate the CoP / ZMP
                auto cops = _cop_estimator.update(com_ref.getValue().head(2),
                    model_joint_pos(left_ankle_name).translation(),
                    model_joint_pos(right_ankle_name).translation(),
                    sensor_data.at("lf_torque"), sensor_data.at("lf_force"),
                    sensor_data.at("rf_torque"), sensor_data.at("rf_force"));

                // if the foot is on the ground
                if (sensor_data.at("lf_force").norm() > _cop_estimator.fmin()) {
                    _lf_force_filtered = _lf_force_filter->filter(sensor_data.at("lf_force"));
                    _lf_torque_filtered = _lf_torque_filter->filter(sensor_data.at("lf_torque"));
                }
                else {
                    _lf_force_filtered.setZero();
                    _lf_torque_filtered.setZero();
                }
                if (sensor_data.at("rf_force").norm() > _cop_estimator.fmin()) {
                    _rf_force_filtered = _rf_force_filter->filter(sensor_data.at("rf_force"));
                    _rf_torque_filtered = _rf_torque_filter->filter(sensor_data.at("rf_torque"));
                }
                else {
                    _rf_force_filtered.setZero();
                    _rf_force_filtered.setZero();
                }

                tsid::trajectories::TrajectorySample lf_se3_sample, lf_contact_sample, rf_se3_sample, rf_contact_sample;
                tsid::trajectories::TrajectorySample com_sample, torso_sample;
                tsid::trajectories::TrajectorySample model_current_com = stabilizer::data_to_sample(tsid_->data());

                const auto& valid_cop = cops[0] ? cops[0] : (cops[1] ? cops[1] : cops[2]);
                // com_admittance
                if (valid_cop) {
                    stabilizer::com_admittance(dt_, _stabilizer_configs[behavior_type_].com_gains, valid_cop.value(), model_current_com, com_ref, com_sample);
                    set_com_ref(com_sample);
                }

                //zmp admittance
                if (valid_cop && _stabilizer_configs[behavior_type_].use_zmp) {

                    double M = pinocchio_total_model_mass();
                    Eigen::Matrix<double, 6, 1> left_fref, right_fref;
                    stabilizer::zmp_distributor_admittance(dt_, _stabilizer_configs[behavior_type_].zmp_p, _stabilizer_configs[behavior_type_].zmp_d,
                        M, contact_se3_ref, ac, valid_cop.value(), model_current_com, left_fref, right_fref);

                    contact("contact_lfoot")->Contact6d::setRegularizationTaskWeightVector(_stabilizer_configs[behavior_type_].zmp_w);
                    contact("contact_rfoot")->Contact6d::setRegularizationTaskWeightVector(_stabilizer_configs[behavior_type_].zmp_w);
                    contact("contact_lfoot")->Contact6d::setForceReference(left_fref);
                    contact("contact_rfoot")->Contact6d::setForceReference(right_fref);
                }

                // left ankle_admittance
                if (cops[1] && std::find(ac.begin(), ac.end(), "contact_lfoot") != ac.end()) {

                    stabilizer::ankle_admittance(dt_, _stabilizer_configs[behavior_type_].ankle_gains, cops[1].value(),
                        model_joint_pos(left_ankle_name), get_full_se3_ref("lf"), contact_sample_ref["contact_lfoot"], lf_se3_sample, lf_contact_sample);
                    set_se3_ref(lf_se3_sample, "lf");
                    contact("contact_lfoot")->setReference(lf_contact_sample);
                }

                //right ankle_admittance
                if (cops[2] && std::find(ac.begin(), ac.end(), "contact_rfoot") != ac.end()) {
                    stabilizer::ankle_admittance(dt_, _stabilizer_configs[behavior_type_].ankle_gains, cops[2].value(),
                        model_joint_pos(right_ankle_name), get_full_se3_ref("rf"), contact_sample_ref["contact_rfoot"], rf_se3_sample, rf_contact_sample);
                    set_se3_ref(rf_se3_sample, "rf");
                    contact("contact_rfoot")->setReference(rf_contact_sample);
                }

                //foot force difference admittance
                if (activated_contacts_forces_.find("contact_rfoot") != activated_contacts_forces_.end()
                    && activated_contacts_forces_.find("contact_lfoot") != activated_contacts_forces_.end()
                    && _lf_force_filter->data_ready()
                    && _rf_force_filter->data_ready()) {

                    //normal force of the contacts from tsid solution
                    double lf_normal_force = contact("contact_lfoot")->Contact6d::getNormalForce(activated_contacts_forces_["contact_lfoot"]);
                    double rf_normal_force = contact("contact_rfoot")->Contact6d::getNormalForce(activated_contacts_forces_["contact_rfoot"]);
                    double M = pinocchio_total_model_mass();

                    stabilizer::foot_force_difference_admittance(dt_, M * 9.81, _stabilizer_configs[behavior_type_].ffda_gains, lf_normal_force,
                        rf_normal_force, _lf_force_filtered, _rf_force_filtered, get_full_se3_ref("torso"), torso_sample);
                    set_se3_ref(torso_sample, "torso");
                }
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
            if (_use_stabilizer) {
                set_com_ref(com_ref);
                set_se3_ref(left_ankle_ref, "lf");
                set_se3_ref(right_ankle_ref, "rf");
                set_se3_ref(torso_ref, "torso");

                for (auto& contact_name : ac) {
                    contact(contact_name)->setReference(contact_sample_ref[contact_name]);
                    contact(contact_name)->Contact6d::setForceReference(contact_force_ref[contact_name]);
                }
            }
        }
    } // namespace controllers
} // namespace inria_wbc
