/* Pinocchio !!!! NEED TO BE INCLUDED BEFORE BOOST*/
#include <pinocchio/algorithm/joint-configuration.hpp> // integrate
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Core>
#include <chrono>
#include <iomanip>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <tsid/contacts/contact-6d-ext.hpp>
#include <tsid/contacts/contact-point.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/math/utils.hpp>
#include <tsid/robots/fwd.hpp>
#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/solvers/solver-HQP-base.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/utils.hpp>
#include <tsid/tasks/task-actuation-bounds.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-joint-bounds.hpp>
#include <tsid/tasks/task-joint-posVelAcc-bounds.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/trajectories/trajectory-base.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>
#include <tsid/trajectories/trajectory-se3.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/utils/stop-watch.hpp>

#include "inria_wbc/controllers/controller.hpp"
#include "inria_wbc/exceptions.hpp"

using namespace tsid;
using namespace tsid::trajectories;
using namespace tsid::math;
using namespace tsid::contacts;
using namespace tsid::tasks;
using namespace tsid::solvers;
using namespace tsid::robots;
using namespace std;
using namespace inria_wbc::utils;

namespace inria_wbc {
    namespace controllers {

        const std::string behavior_types::FIXED_BASE = "fixed_base";
        const std::string behavior_types::SINGLE_SUPPORT = "single_support";
        const std::string behavior_types::DOUBLE_SUPPORT = "double_support";

        Controller::Controller(const YAML::Node& config) : config_(config)
        {
            auto c = IWBC_CHECK(config["CONTROLLER"]);
            base_path_ = IWBC_CHECK(c["base_path"].as<std::string>());
            auto floating_base_joint_name = IWBC_CHECK(c["floating_base_joint_name"].as<std::string>());
            auto urdf = IWBC_CHECK(c["urdf"].as<std::string>());
            dt_ = IWBC_CHECK(c["dt"].as<double>());
            mimic_dof_names_ = IWBC_CHECK(c["mimic_dof_names"].as<std::vector<std::string>>());
            verbose_ = IWBC_CHECK(c["verbose"].as<bool>());

            // closed loop
            _closed_loop = IWBC_CHECK(c["closed_loop"].as<bool>());

            pinocchio::Model robot_model;
            floating_base_ = IWBC_CHECK(c["floating_base"].as<bool>());

            if (floating_base_) {
                if (!floating_base_joint_name.empty()) {
                    fb_joint_name_ = floating_base_joint_name; //floating base joint already in urdf
                    pinocchio::urdf::buildModel(urdf, robot_model, verbose_);
                }
                else {
                    pinocchio::urdf::buildModel(urdf, pinocchio::JointModelFreeFlyer(), robot_model, verbose_);
                    fb_joint_name_ = "root_joint";
                }
                robot_ = std::make_shared<RobotWrapper>(robot_model, verbose_);
            }
            else {
                fb_joint_name_ = "";
                robot_ = std::make_shared<RobotWrapper>(urdf, std::vector<std::string>(), verbose_); //this overloaded constructor allows to to not have a f_base
            }

            if (verbose_) {
                std::cout << "--------- Robot size info ---------" << std::endl;
                std::cout << "Number of actuators : " << robot_->na() << std::endl;
                std::cout << "Dimension of the configuration vector : " << robot_->nq() << std::endl;
                std::cout << "Dimension of the velocity vector : " << robot_->nv() << std::endl;
                std::cout << "--------- ------------- ---------" << std::endl;
                std::cout << "position tracker initializer" << std::endl;
            }

            _reset();
        }

        // reset everything (called from the constructor)
        void Controller::_reset()
        {
            t_ = 0.0;

            uint nactuated = robot_->na();
            uint ndofs = robot_->nv(); // na + 6 (floating base), if not f_base only na

            v_tsid_ = Vector::Zero(ndofs);
            a_tsid_ = Vector::Zero(ndofs);
            tau_tsid_ = Vector::Zero(nactuated);
            momentum_ = Vector::Zero(3);

            q0_.resize(ndofs);
            q_ = Vector::Zero(ndofs);
            dq_ = Vector::Zero(ndofs);
            ddq_ = Vector::Zero(ndofs);
            tau_ = Vector::Zero(ndofs);

            tsid_joint_names_ = all_dofs(false);
            non_mimic_indexes_ = get_non_mimics_indexes();
        }

        void Controller::update(const SensorData& sensor_data)
        {
            if (_closed_loop) {

                IWBC_ASSERT(sensor_data.find("positions") != sensor_data.end(),
                    "we need the joint positions in closed loop mode!");
                IWBC_ASSERT(sensor_data.find("joint_velocities") != sensor_data.end(),
                    "we need the joint velocities in closed loop mode!");
                    Eigen::VectorXd q_tsid(q_tsid_.size()), dq(v_tsid_.size());

                auto pos = sensor_data.at("positions");
                auto vel = sensor_data.at("joint_velocities");

                if (floating_base_)
                {
                    IWBC_ASSERT(sensor_data.find("floating_base_position") != sensor_data.end(),
                        "we need the floating base position in closed loop mode!");
                    IWBC_ASSERT(sensor_data.find("floating_base_velocity") != sensor_data.end(),
                        "we need the floating base velocity in closed loop mode!");

                    Eigen::VectorXd q_tsid(q_tsid_.size()), dq(v_tsid_.size());
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
                else
                {
                    IWBC_ASSERT(vel.size() == v_tsid_.size(),
                        "Joint velocities do not have the correct size:", vel.size(), " vs (expected)", v_tsid_.size());
                    IWBC_ASSERT(pos.size() == q_tsid_.size(),
                        "Joint positions do not have the correct size:", pos.size(), " vs (expected)", q_tsid_.size());

                    _solve(pos, vel);
                }
            }
            else
            {
                _solve(); 
            }
            
        };

        std::vector<int> Controller::get_non_mimics_indexes() const
        {
            std::vector<int> non_mimic_indexes;
            std::vector<int> mimic_indexes;
            if (!mimic_dof_names_.empty()) {
                for (auto& m : mimic_dof_names_) {
                    auto it = std::find(tsid_joint_names_.begin(), tsid_joint_names_.end(), m);
                    IWBC_ASSERT(it != tsid_joint_names_.end(), " joint ", m, " not found");
                    mimic_indexes.push_back(std::distance(tsid_joint_names_.begin(), it));
                }
                for (int i = 0; i < q_.size(); i++) {
                    auto it = std::find(mimic_indexes.begin(), mimic_indexes.end(), i);
                    if (it == mimic_indexes.end())
                        non_mimic_indexes.push_back(i);
                }
            }
            else {
                non_mimic_indexes.resize(q_.size());
                std::generate(non_mimic_indexes.begin(), non_mimic_indexes.end(), [n = 0]() mutable { return n++; });
            }
            return non_mimic_indexes;
        }

        void Controller::_solve(const Eigen::VectorXd& q, const Eigen::VectorXd& dq)
        {
            //Compute the current data from the current position and solve to find next position
            assert(tsid_);
            assert(robot_);
            assert(solver_);
            const HQPData& HQPData = tsid_->computeProblemData(t_, q, dq);
            momentum_ = (robot_->momentumJacobian(tsid_->data()).bottomRows(3) * dq);

            const HQPOutput& sol = solver_->solve(HQPData);

            if (sol.status == HQP_STATUS_OPTIMAL) {
                const Vector& tau = tsid_->getActuatorForces(sol);
                const Vector& dv = tsid_->getAccelerations(sol);
                tau_tsid_ = tau;
                a_tsid_ = dv;
                v_tsid_ = dq + dt_ * dv;
                q_tsid_ = pinocchio::integrate(robot_->model(), q, dt_ * v_tsid_);
                t_ += dt_;

                activated_contacts_forces_.clear();
                for (auto& contact : activated_contacts_) {
                    activated_contacts_forces_[contact] = tsid_->getContactForces(contact, sol);
                }

                if (floating_base_) {
                    Eigen::Quaterniond quat(q_tsid_(6), q_tsid_(3), q_tsid_(4), q_tsid_(5));
                    Eigen::AngleAxisd aaxis(quat);
                    q_ << q_tsid_.head(3), aaxis.angle() * aaxis.axis(), q_tsid_.tail(robot_->nq() - 7); //q_tsid_ of size 37 (pos+quat+nactuated)
                    tau_ << 0, 0, 0, 0, 0, 0, tau_tsid_; //the size of tau is actually 30 (nactuated)
                }
                else {
                    q_ << q_tsid_;
                    tau_ << tau_tsid_;
                }

                dq_ = v_tsid_; //the speed of the free flyerjoint is dim 6 even if its pos id dim 7
                ddq_ = a_tsid_;
            }
            else {
                std::string error = "Controller failed, can't solve problem. ";
                error += "Status : " + toString(sol.status);
                switch (sol.status) {
                case -1:
                    error += " => Unknown";
                    break;
                case 1:
                    error += " => Infeasible ";
                    break;
                case 2:
                    error += " => Unbounded ";
                    break;
                case 3:
                    error += " => Max iter reached ";
                    break;
                case 4:
                    error += " => Error ";
                    break;
                default:
                    error += " => Uknown status";
                }
                error += " (t=" + std::to_string(t_) + ")";
                throw IWBC_EXCEPTION(error);
            }
        }

        // Removes the universe and root (floating base) joint names
        std::vector<std::string> Controller::controllable_dofs(bool filter_mimics) const
        {
            auto na = robot_->model().names;
            std::vector<std::string> tsid_controllables;
            if (floating_base_) {
                tsid_controllables = std::vector<std::string>(robot_->model().names.begin() + 2, robot_->model().names.end());
            }
            else {
                tsid_controllables = std::vector<std::string>(robot_->model().names.begin() + 1, robot_->model().names.end());
            }

            if (filter_mimics)
                return remove_intersection(tsid_controllables, mimic_dof_names_);
            else
                return tsid_controllables;
        }

        // Order of the floating base in q_ according to dart naming convention
        std::vector<std::string> Controller::floating_base_dofs() const
        {

            std::vector<std::string> floating_base_dofs;
            if (floating_base_) {

                if (fb_joint_name_ == "root_joint") {
                    floating_base_dofs = {"rootJoint_pos_x",
                        "rootJoint_pos_y",
                        "rootJoint_pos_z",
                        "rootJoint_rot_x",
                        "rootJoint_rot_y",
                        "rootJoint_rot_z"};
                }
                else {
                    floating_base_dofs = {fb_joint_name_ + "_pos_x",
                        fb_joint_name_ + "_pos_y",
                        fb_joint_name_ + "_pos_z",
                        fb_joint_name_ + "_rot_x",
                        fb_joint_name_ + "_rot_y",
                        fb_joint_name_ + "_rot_z"};
                }
            }
            else {
                floating_base_dofs = {};
            }

            return floating_base_dofs;
        }

        std::vector<std::string> Controller::all_dofs(bool filter_mimics) const
        {
            std::vector<std::string> all_dofs = floating_base_dofs();
            std::vector<std::string> control_dofs = controllable_dofs(filter_mimics);
            all_dofs.insert(all_dofs.end(), control_dofs.begin(), control_dofs.end());
            return filter_mimics ? remove_intersection(all_dofs, mimic_dof_names_) : all_dofs;
        }

        Eigen::VectorXd Controller::tau(bool filter_mimics) const
        {
            return filter_mimics ? slice_vec(tau_, non_mimic_indexes_) : tau_;
        }

        Eigen::VectorXd Controller::ddq(bool filter_mimics) const
        {
            return filter_mimics ? slice_vec(ddq_, non_mimic_indexes_) : ddq_;
        }

        Eigen::VectorXd Controller::dq(bool filter_mimics) const
        {
            return filter_mimics ? slice_vec(dq_, non_mimic_indexes_) : dq_;
        }

        Eigen::VectorXd Controller::q(bool filter_mimics) const
        {
            return filter_mimics ? slice_vec(q_, non_mimic_indexes_) : q_;
        }

        Eigen::VectorXd Controller::q0(bool filter_mimics) const
        {
            return filter_mimics ? slice_vec(q0_, non_mimic_indexes_) : q0_;
        }

        void Controller::save_configuration(const std::string config_name, const std::string robot_name) const
        {
            std::ofstream config(config_name);
            config << "<?xml version=\"1.0\" ?>" << std::endl;
            config << "<robot name=\"" << robot_name << "\">" << std::endl;
            config << "\t<group_state name=\"" << config_name.substr(0, config_name.length() - strlen(".srdf")) << "\" group=\"all\">" << std::endl;
            config << "\t\t<joint name=\"root_joint\" value=\"" << q_tsid_.head(7).transpose() << "\" />" << std::endl;
            auto names = all_dofs(false);
            for (uint i = 6; i < names.size(); i++) {
                config << "\t\t<joint name=\"" << names[i] << "\" value=\"" << q_[i] << "\" />" << std::endl;
            }
            config << "\t</group_state>" << std::endl;
            config << "</robot>" << std::endl;
        }

        std::vector<double> Controller::pinocchio_model_masses() const
        {
            std::vector<double> masses;
            for (int i = 0; i < robot_->model().inertias.size(); i++) {
                //order corresponds to robot_->model().names
                masses.push_back(robot_->model().inertias[i].mass());
            }
            return masses;
        }

        double Controller::pinocchio_total_model_mass() const
        {
            double mass = 0.0;
            for (int i = 0; i < robot_->model().inertias.size(); i++) {
                mass += robot_->model().inertias[i].mass();
            }
            return mass;
        }

        void Controller::set_behavior_type(std::string bt)
        {
            if (bt != behavior_types::FIXED_BASE && bt != behavior_types::SINGLE_SUPPORT && bt != behavior_types::DOUBLE_SUPPORT)
                IWBC_ERROR("behavior type is either behavior_types::FIXED_BASE , behavior_types::SINGLE_SUPPORT or behavior_types::DOUBLE_SUPPORT ");
            behavior_type_ = bt;
            parse_stabilizer(config_["CONTROLLER"]);
        }

    } // namespace controllers
} // namespace inria_wbc
