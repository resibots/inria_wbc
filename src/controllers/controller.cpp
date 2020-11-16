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

#include <tsid/contacts/contact-6d.hpp>
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
        Controller::Controller(const Params& params)
        {
            params_ = params;
            verbose_ = params_.verbose;
            pinocchio::Model robot_model;
            if (!params.floating_base_joint_name.empty()) {
                fb_joint_name_ = params.floating_base_joint_name; //floating base joint already in urdf
                pinocchio::urdf::buildModel(params.urdf_path, robot_model, verbose_);
            }
            else {
                pinocchio::urdf::buildModel(params.urdf_path, pinocchio::JointModelFreeFlyer(), robot_model, verbose_);
                fb_joint_name_ = "root_joint";
            }
            robot_ = std::make_shared<RobotWrapper>(robot_model, verbose_);
            pinocchio::srdf::loadReferenceConfigurations(robot_->model(), params.srdf_path, verbose_); //the srdf contains initial joint positions

            _reset();
        }

        // reset everything (called from the constructor)
        void Controller::_reset()
        {
            dt_ = params_.dt;
            mimic_dof_names_ = params_.mimic_dof_names;
            t_ = 0.0;

            uint nactuated = robot_->na();
            uint ndofs = robot_->nv(); // na + 6 (floating base)

            v_tsid_ = Vector::Zero(ndofs);
            a_tsid_ = Vector::Zero(ndofs);
            tau_tsid_ = Vector::Zero(nactuated);

            q0_.resize(ndofs);
            q_ = Vector::Zero(ndofs);
            dq_ = Vector::Zero(ndofs);
            ddq_ = Vector::Zero(ndofs);
            tau_ = Vector::Zero(ndofs);

            tsid_joint_names_ = all_dofs(false);
            non_mimic_indexes_ = get_non_mimics_indexes();
        }

        std::vector<int> Controller::get_non_mimics_indexes() const
        {
            std::vector<int> non_mimic_indexes;
            std::vector<int> mimic_indexes;
            if (!mimic_dof_names_.empty()) {
                for (auto& m : mimic_dof_names_) {
                    auto it = std::find(tsid_joint_names_.begin(), tsid_joint_names_.end(), m);
                    IWBC_ASSERT(it != tsid_joint_names_.end(), " joint ", *it, " not found");
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

        void Controller::_solve()
        {
            //Compute the current data from the current position and solve to find next position
            assert(tsid_);
            assert(robot_);
            assert(solver_);

            const HQPData& HQPData = tsid_->computeProblemData(t_, q_tsid_, v_tsid_);

            const HQPOutput& sol = solver_->solve(HQPData);

            if (sol.status == HQP_STATUS_OPTIMAL) {
                const Vector& tau = tsid_->getActuatorForces(sol);
                const Vector& dv = tsid_->getAccelerations(sol);
                tau_tsid_ = tau;
                a_tsid_ = dv;
                v_tsid_ += dt_ * dv;

                q_tsid_ = pinocchio::integrate(robot_->model(), q_tsid_, dt_ * v_tsid_);
                t_ += dt_;

                Eigen::Quaterniond quat(q_tsid_(6), q_tsid_(3), q_tsid_(4), q_tsid_(5));
                Eigen::AngleAxisd aaxis(quat);
                q_ << q_tsid_.head(3), aaxis.angle() * aaxis.axis(), q_tsid_.tail(robot_->nq() - 7); //q_tsid_ of size 37 (pos+quat+nactuated)
                dq_ = v_tsid_; //the speed of the free flyerjoint is dim 6 even if its pos id dim 7
                tau_ << 0, 0, 0, 0, 0, 0, tau_tsid_; //the size of tau is actually 30 (nactuated)
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
                throw IWBC_EXCEPTION(error);
            }
        }

        // Removes the universe and root (floating base) joint names
        std::vector<std::string> Controller::controllable_dofs(bool filter_mimics) const
        {
            auto na = robot_->model().names;
            auto tsid_controllables = std::vector<std::string>(robot_->model().names.begin() + 2, robot_->model().names.end());
            if (filter_mimics)
                return remove_intersection(tsid_controllables, mimic_dof_names_);
            else
                return tsid_controllables;
        }

        // Order of the floating base in q_ according to dart naming convention
        std::vector<std::string> Controller::floating_base_dofs() const
        {
            std::vector<std::string> floating_base_dofs;
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

            return floating_base_dofs;
        }

        std::vector<std::string> Controller::all_dofs(bool filter_mimics) const
        {
            std::vector<std::string> all_dofs = floating_base_dofs();
            std::vector<std::string> control_dofs = controllable_dofs(filter_mimics);
            all_dofs.insert(all_dofs.end(), control_dofs.begin(), control_dofs.end());
            return filter_mimics ? remove_intersection(all_dofs, mimic_dof_names_) : all_dofs;
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

        std::vector<double> Controller::pinocchio_model_masses() const
        {
            std::vector<double> masses;
            for (int i = 0; i < robot_->model().inertias.size(); i++) {
                //order corresponds to robot_->model().names
                masses.push_back(robot_->model().inertias[i].mass());
            }
            return masses;
        }

        std::shared_ptr<tasks::TaskComEquality> Controller::make_com_task(const std::string& name, double kp) const
        {
            assert(tsid_);
            assert(robot_);
            auto task = std::make_shared<tasks::TaskComEquality>(name, *robot_);
            task->Kp(kp * Vector::Ones(3));
            task->Kd(2.0 * task->Kp().cwiseSqrt());
            task->setReference(to_sample(robot_->com(tsid_->data())));
            return task;
        }

        std::shared_ptr<tasks::TaskJointPosture> Controller::make_posture_task(const std::string& name, double kp, const Vector& mask) const
        {
            assert(tsid_);
            assert(robot_);
            auto task = std::make_shared<tasks::TaskJointPosture>(name, *robot_);
            task->Kp(kp * Vector::Ones(robot_->nv() - 6));
            task->Kd(2.0 * task->Kp().cwiseSqrt());
            Vector mask_post(robot_->nv() - 6);
            if (mask.size() == 0) {
                mask_post = Vector::Ones(robot_->nv() - 6);
            }
            else {
                assert(mask.size() == mask_post.size());
                mask_post = mask;
            }
            task->setMask(mask_post);
            task->setReference(to_sample(q_tsid_.tail(robot_->na())));
            return task;
        }

        std::shared_ptr<tasks::TaskSE3Equality> Controller::make_se3_frame_task(const std::string& name, const std::string& frame_name, double kp, const se3_mask::mask6& mask) const
        {
            assert(tsid_);
            assert(robot_);
            auto task = std::make_shared<tasks::TaskSE3Equality>(name, *robot_, frame_name);
            task->Kp(kp * Vector::Ones(6));
            task->Kd(2.0 * task->Kp().cwiseSqrt());
            task->setMask(mask);
            auto ref = robot_->framePosition(tsid_->data(), robot_->model().getFrameId(frame_name));
            auto sample = to_sample(ref);
            task->setReference(sample);
            return task;
        }

        std::shared_ptr<tasks::TaskSE3Equality> Controller::make_se3_joint_task(const std::string& name, const std::string& joint_name, double kp, const se3_mask::mask6& mask) const
        {
            assert(tsid_);
            assert(robot_);
            auto task = std::make_shared<tasks::TaskSE3Equality>(name, *robot_, joint_name);
            task->Kp(kp * Vector::Ones(6));
            task->Kd(2.0 * task->Kp().cwiseSqrt());
            task->setMask(mask.matrix());
            auto ref = robot_->position(tsid_->data(), robot_->model().getJointId(joint_name));
            auto sample = to_sample(ref);
            task->setReference(sample);
            return task;
        }

        std::shared_ptr<tasks::TaskJointPosVelAccBounds> Controller::make_bound_task(const std::string& name) const
        {
            assert(tsid_);
            assert(robot_);
            auto task = std::make_shared<tasks::TaskJointPosVelAccBounds>(name, *robot_, dt_, verbose_);
            auto dq_max = robot_->model().velocityLimit.tail(robot_->na());
            auto ddq_max = dq_max / dt_;
            task->setVelocityBounds(dq_max);
            task->setAccelerationBounds(ddq_max);
            auto q_lb = robot_->model().lowerPositionLimit.tail(robot_->na());
            auto q_ub = robot_->model().upperPositionLimit.tail(robot_->na());
            task->setPositionBounds(q_lb, q_ub);
            return task;
        }

        inria_wbc::controllers::Controller::Params parse_params(YAML::Node config)
        {
            std::string urdf_path = "";
            std::string srdf_path = "";
            std::string sot_config_path = "";
            std::string floating_base_joint_name = "";
            float dt = 0.001;
            bool verbose = false;
            std::vector<std::string> mimic_dof_names = {};
            parse(urdf_path, "urdf_path", config, "PARAMS", verbose);
            parse(srdf_path, "srdf_path", config, "PARAMS", verbose);
            parse(sot_config_path, "sot_config_path", config, "PARAMS", verbose);
            parse(floating_base_joint_name, "floating_base_joint_name", config, "PARAMS", verbose);
            parse(dt, "dt", config, "PARAMS", verbose);
            parse(verbose, "verbose", config, "PARAMS", verbose);
            parse(mimic_dof_names, "mimic_dof_names", config, "PARAMS", verbose);

            Controller::Params params = {urdf_path,
                srdf_path,
                sot_config_path,
                floating_base_joint_name,
                dt,
                verbose,
                mimic_dof_names};

            return params;
        }

    } // namespace controllers
} // namespace inria_wbc
