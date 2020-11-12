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

#include "inria_wbc/controllers/ex_controller.hpp"

using namespace tsid;
using namespace tsid::math;

namespace inria_wbc {
    namespace controllers {
        static Register<ExController> __talos_pos_tracking("ex-controller");

        ExController::ExController(const Params& params) : TalosPosTracking(params)
        {
            if (!params.sot_config_path.empty())
                parse_configuration_yaml(params.sot_config_path);

            set_stack_configuration();
        }

        void ExController::set_default_opt_params(std::map<std::string, double>& p)
        {
            TalosPosTracking::set_default_opt_params(p);
            // you can add some parameters here
        }

        void ExController::parse_configuration_yaml(const std::string& sot_config_path)
        {
            TalosPosTracking::parse_configuration_yaml(sot_config_path);
            // you can re-open the file here and parse additional parameters
        }

        void ExController::set_stack_configuration()
        {
            // for the example, we rebuild a full stack here
            // with the new TSID example task

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

            auto vert_torso_task = make_se3_frame_task("torso", cst::torso_frame_name, p.at("kp_torso"), se3_mask::roll + se3_mask::pitch);
            if (p.at("w_torso") > 0)
                tsid_->addMotionTask(*vert_torso_task, p.at("w_torso"), 1);
            se3_tasks_[vert_torso_task->name()] = vert_torso_task;

            auto floatingb_task = make_se3_joint_task("floatingb", fb_joint_name_, p.at("kp_floatingb"), inria_wbc::se3_mask::rpy);
            if (p.at("w_floatingb") > 0)
                tsid_->addMotionTask(*floatingb_task, p.at("w_floatingb"), 1);
            se3_tasks_[floatingb_task->name()] = floatingb_task;

            auto lh_task = make_se3_joint_task("lh", cst::lh_joint_name, p.at("kp_lh"), inria_wbc::se3_mask::xyz);
            if (p.at("w_lh") > 0)
                tsid_->addMotionTask(*lh_task, p.at("w_lh"), 1);
            se3_tasks_[lh_task->name()] = lh_task;

            auto rh_task = make_se3_joint_task("rh", cst::rh_joint_name, p.at("kp_rh"), inria_wbc::se3_mask::xyz);
            if (p.at("w_rh") > 0)
                tsid_->addMotionTask(*rh_task, p.at("w_rh"), 1);
            se3_tasks_[rh_task->name()] = rh_task;

            auto lf_task = make_se3_joint_task("lf", cst::lf_joint_name, p.at("kp_lf"), inria_wbc::se3_mask::all);
            if (p.at("w_lf") > 0)
                tsid_->addMotionTask(*lf_task, p.at("w_lf"), 1);
            se3_tasks_[lf_task->name()] = lf_task;

            auto rf_task = make_se3_joint_task("rf", cst::rf_joint_name, p.at("kp_rf"), inria_wbc::se3_mask::all);
            if (p.at("w_rf") > 0)
                tsid_->addMotionTask(*rf_task, p.at("w_rf"), 1);
            se3_tasks_[rf_task->name()] = rf_task;
        }

    } // namespace controllers
} // namespace inria_wbc
