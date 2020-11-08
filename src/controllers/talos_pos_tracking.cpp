/* Pinocchio !!!! NEED TO BE INCLUDED BEFORE BOOST*/
#include <pinocchio/algorithm/joint-configuration.hpp> // integrate
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "Eigen/Core"
#include "map"
#include "vector"
#include <iomanip>
#include <memory>
#include <utility>

#include <tsid/solvers/solver-HQP-base.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/utils.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/utils/stop-watch.hpp>

#include "inria_wbc/controllers/talos_pos_tracking.hpp"

using namespace tsid;
using namespace tsid::math;

namespace inria_wbc {
    namespace controllers {
        static Register<TalosPosTracking> __talos_pos_tracking("talos-pos-tracking");

        TalosPosTracking::TalosPosTracking(const Params& params) : Controller(params)
        {
            if (!params.sot_config_path.empty())
                parse_configuration_yaml(params.sot_config_path);

            set_stack_configuration();
        }

        void TalosPosTracking::set_default_opt_params(std::map<std::string, double>& p)
        {
            p["w_com"] = 10.0; //# weight of center of mass task
            p["w_posture"] = 0.75; //# weight of joint posture task
            p["w_forceRef_feet"] = 1e-3; //# weight of force regularization task
            p["w_forceRef_hands"] = 1e-3; //# weight of force regularization task
            p["w_floatingb"] = 20.0; //# weight of floatingb task
            p["w_velocity"] = 1.0; //# weight of velocity bounds
            p["w_rh"] = 10.0; //# weight of right hand  task
            p["w_lh"] = 10.0; //# weight of left hand  task
            p["w_rf"] = 1.0; //# weight of right foot  task
            p["w_lf"] = 1.0; //# weight of left foot  task
            p["w_torso"] = 1000.0; //# weight of torso task

            p["kp_contact"] = 30.0; //# proportional gain of contact constraint
            p["kp_com"] = 3000.0; //# proportional gain of center of mass task
            p["kp_posture"] = 30.0; //# proportional gain of joint posture task
            p["kp_floatingb"] = 3000.0; //# proportional gain of floatingb task
            p["kp_rh"] = 300.0; //# proportional gain of right hand task
            p["kp_lh"] = 300.0; //# proportional gain of left hand task
            p["kp_rf"] = 30.0; //# proportional gain of right foot task
            p["kp_lf"] = 30.0; //# proportional gain of left foot task
            p["kp_torso"] = 30.0; //# proportional gain of the torso task
        }

        void TalosPosTracking::parse_configuration_yaml(const std::string& sot_config_path)
        {
            std::ifstream yamlConfigFile(sot_config_path);
            if (!yamlConfigFile.good()) {
                if (verbose_) {
                    std::cout << sot_config_path << " not found" << std::endl;
                    std::cout << "Taking default stack of tasks weights" << std::endl;
                }
            }
            else {
                if (verbose_)
                    std::cout << "Taking the stack of tasks parameters in " << sot_config_path << std::endl;

                // for the opt_params (task weight and gains) we keep the value if we have it
                // if not found, we look at the YAML file
                // if not found, we take the default value from the map (set_defautlt_opt_params)
                opt_params_t p;
                set_default_opt_params(p);
                YAML::Node config = YAML::LoadFile(sot_config_path);
                for (auto& x : p)
                    if (params_.opt_params.find(x.first) == params_.opt_params.end())
                        if (!utils::parse(params_.opt_params[x.first], x.first, config, verbose_))
                            params_.opt_params[x.first] = p[x.first];
            }
        }

        std::shared_ptr<contacts::Contact6d> TalosPosTracking::make_contact_task(const std::string& name, const std::string frame_name, double kp) const
        {
            assert(tsid_);
            assert(robot_);
            Matrix3x contact_points(3, 4);
            contact_points << -cst::lxn, -cst::lxn, cst::lxp, cst::lxp,
                -cst::lyn, cst::lyp, -cst::lyn, cst::lyp,
                cst::lz, cst::lz, cst::lz, cst::lz;
            auto contact_task = std::make_shared<contacts::Contact6d>(name, *robot_, frame_name, contact_points, cst::contact_normal, cst::mu, cst::fMin, cst::fMax);
            contact_task->Kp(kp * Vector::Ones(6));
            contact_task->Kd(2.0 * contact_task->Kp().cwiseSqrt());
            auto contact_ref = robot_->position(tsid_->data(), robot_->model().getJointId(frame_name));
            contact_task->setReference(contact_ref);
            return contact_task;
        }

        void TalosPosTracking::set_stack_configuration()
        {

            const opt_params_t& p = params_.opt_params;
            if (verbose_)
                for (auto& x : p)
                    std::cout << x.first << " => " << params_.opt_params[x.first] << std::endl;

            ////////////////////Gather Initial Pose //////////////////////////////////////
            //q_tsid_ is of size 37 (pos+quat+nactuated)
            q_tsid_ = robot_->model().referenceConfigurations["pal_start"];
            //q0_ is in "Dart format" for the floating base
            Eigen::Quaterniond quat(q_tsid_(6), q_tsid_(3), q_tsid_(4), q_tsid_(5));
            Eigen::AngleAxisd aaxis(quat);
            q0_ << q_tsid_.head(3), aaxis.angle() * aaxis.axis(), q_tsid_.tail(robot_->na());

            ////////////////////Create the inverse-dynamics formulation///////////////////
            tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);

            ////////////////////Create an HQP solver /////////////////////////////////////
            using solver_t = std::shared_ptr<solvers::SolverHQPBase>;
            solver_ = solver_t(solvers::SolverHQPFactory::createNewSolver(solvers::SOLVER_HQP_EIQUADPROG_FAST, "solver-eiquadprog"));
            solver_->resize(tsid_->nVar(), tsid_->nEq(), tsid_->nIn());

            ////////////////////Compute Problem Data at init /////////////////////////////
            const uint nv = robot_->nv();
            tsid_->computeProblemData(dt_, q_tsid_, Vector::Zero(nv));

            assert(tsid_);
            assert(robot_);
            ////////////////////Compute Tasks, Bounds and Contacts ///////////////////////
            contactRF_ = make_contact_task("contact_rfoot", cst::rf_joint_name, p.at("kp_contact"));
            if (p.at("w_forceRef_feet") > 0)
                tsid_->addRigidContact(*contactRF_, p.at("w_forceRef_feet"));

            contactLF_ = make_contact_task("contact_lfoot", cst::lf_joint_name, p.at("kp_contact"));
            if (p.at("w_forceRef_feet") > 0)
                tsid_->addRigidContact(*contactLF_, p.at("w_forceRef_feet"));

            com_task_ = make_com_task("com", p.at("kp_com"));
            if (p.at("w_com") > 0)
                tsid_->addMotionTask(*com_task_, p.at("w_com"), 1);

            posture_task_ = make_posture_task("posture", p.at("kp_posture"));
            if (p.at("w_posture") > 0)
                tsid_->addMotionTask(*posture_task_, p.at("w_posture"), 1);

            bounds_task_ = make_bound_task("task-posVelAcc-bounds");
            if (p.at("w_velocity") > 0)
                tsid_->addMotionTask(*bounds_task_, p.at("w_velocity"), 0);

            auto vert_torso_task = make_se3_frame_task("torso", cst::torso_frame_name, p.at("kp_torso"), mask::roll + mask::pitch);
            if (p.at("w_torso") > 0)
                tsid_->addMotionTask(*vert_torso_task, p.at("w_torso"), 1);
            se3_tasks_[vert_torso_task->name()] = vert_torso_task;

            auto floatingb_task = make_se3_task("floatingb", fb_joint_name_, p.at("kp_floatingb"), inria_wbc::mask::rpy);
            if (p.at("w_floatingb") > 0)
                tsid_->addMotionTask(*floatingb_task, p.at("w_floatingb"), 1);
            se3_tasks_[floatingb_task->name()] = floatingb_task;

            auto lh_task = make_se3_task("lh", cst::lh_joint_name, p.at("kp_lh"), inria_wbc::mask::xyz);
            if (p.at("w_lh") > 0)
                tsid_->addMotionTask(*lh_task, p.at("w_lh"), 1);
            se3_tasks_[lh_task->name()] = lh_task;

            auto rh_task = make_se3_task("rh", cst::rh_joint_name, p.at("kp_rh"), inria_wbc::mask::xyz);
            if (p.at("w_rh") > 0)
                tsid_->addMotionTask(*rh_task, p.at("w_rh"), 1);
            se3_tasks_[rh_task->name()] = rh_task;

            auto lf_task = make_se3_task("lf", cst::lf_joint_name, p.at("kp_lf"), inria_wbc::mask::all);
            if (p.at("w_lf") > 0)
                tsid_->addMotionTask(*lf_task, p.at("w_lf"), 1);
            se3_tasks_[lf_task->name()] = lf_task;

            auto rf_task = make_se3_task("rf", cst::rf_joint_name, p.at("kp_rf"), inria_wbc::mask::all);
            if (p.at("w_rf") > 0)
                tsid_->addMotionTask(*rf_task, p.at("w_rf"), 1);
            se3_tasks_[rf_task->name()] = rf_task;
        }

        pinocchio::SE3 TalosPosTracking::get_se3_ref(const std::string& task_name)
        {
            auto it = se3_tasks_.find(task_name);
            assert(it != se3_tasks_.end());
            pinocchio::SE3 se3;
            auto pos = it->second->getReference().pos;
            tsid::math::vectorToSE3(pos, se3);
            return se3;
        }

        void TalosPosTracking::set_se3_ref(const pinocchio::SE3& ref, const std::string& task_name)
        {
            auto it = se3_tasks_.find(task_name);
            assert(it != se3_tasks_.end());
            auto sample = to_sample(ref);
            it->second->setReference(sample);
        }

        void TalosPosTracking::set_com_ref(const Vector3& ref)
        {
            com_task_->setReference(to_sample(ref));
        }

        void TalosPosTracking::set_posture_ref(const tsid::math::Vector& ref)
        {
            posture_task_->setReference(to_sample(ref));
        }

        void TalosPosTracking::remove_contact(const std::string& contact_name)
        {
            tsid_->removeRigidContact(contact_name);
        }

        void TalosPosTracking::add_contact(const std::string& contact_name)
        {
            const opt_params_t& p = params_.opt_params;
            const pinocchio::Data& data = tsid_->data();
            if (contact_name == "contact_rfoot") {
                auto contact_rf_ref = robot_->position(data, robot_->model().getJointId(cst::rf_joint_name));
                contactRF_->setReference(contact_rf_ref);
                tsid_->addRigidContact(*contactRF_, p.at("w_forceRef_feet"));
            }
            else if (contact_name == "contact_lfoot") {
                auto contact_lf_ref = robot_->position(data, robot_->model().getJointId(cst::lf_joint_name));
                contactLF_->setReference(contact_lf_ref);
                tsid_->addRigidContact(*contactLF_, p.at("w_forceRef_feet"));
            }
            else {
                std::cout << "unknown contact" << std::endl;
            }
        }

    } // namespace controllers
} // namespace inria_wbc
