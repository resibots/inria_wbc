/* Pinocchio !!!! NEED TO BE INCLUDED BEFORE BOOST*/
#include <pinocchio/algorithm/joint-configuration.hpp> // integrate
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "Eigen/Core"
#include <iomanip>
#include <memory>
#include <utility>
#include "map"
#include "string"
#include "vector"
#include "utils.hpp"
#include <unordered_map>


#include <tsid/contacts/contact-6d.hpp>
#include <tsid/contacts/contact-point.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-actuation-bounds.hpp>
#include <tsid/tasks/task-joint-bounds.hpp>
#include <tsid/tasks/task-joint-posVelAcc-bounds.hpp>
#include <tsid/trajectories/trajectory-base.hpp>
#include <tsid/trajectories/trajectory-se3.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-HQP-base.hpp>
#include <tsid/solvers/utils.hpp>
#include <tsid/utils/stop-watch.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/math/utils.hpp>
#include <tsid/robots/fwd.hpp>
#include <tsid/robots/robot-wrapper.hpp>


#include "talos_pos_tracking.hpp"

using namespace tsid;
using namespace tsid::trajectories;
using namespace tsid::math;
using namespace tsid::contacts;
using namespace tsid::tasks;
using namespace tsid::solvers;
using namespace tsid::robots;
using namespace std;

namespace tsid_sot
{
    TalosPosTracking::TalosPosTracking(const Params &params, 
                                       const std::string &sot_config_path, 
                                       const std::string& fb_joint_name, 
                                       const std::vector<std::string> &mimic_joint_names, 
                                       bool verbose)
    {
      dt_ = params.dt;
      verbose_ = verbose;
      t_ = 0.0;

      pinocchio::Model robot_model;
      if(!fb_joint_name.empty())
      {
        fb_joint_name_ = fb_joint_name;//floating base joint already in urdf
        pinocchio::urdf::buildModel(params.urdf_path, robot_model, verbose_);
      }
      else
      {
        pinocchio::urdf::buildModel(params.urdf_path, pinocchio::JointModelFreeFlyer(), robot_model, verbose_); 
        fb_joint_name_ = "root_joint";
      }
      robot_ = std::make_shared<RobotWrapper>(robot_model, verbose_);
      pinocchio::srdf::loadReferenceConfigurations(robot_->model(), params.srdf_path, verbose_); //the srdf contains initial joint positions

      uint nactuated = robot_->na();
      uint ndofs = robot_->nv(); // na + 6 (floating base)

      v_tsid_ = Vector::Zero(ndofs);
      a_tsid_ = Vector::Zero(ndofs);
      tau_tsid_ = Vector::Zero(nactuated);
      q_lb_ = Vector::Zero(nactuated);
      q_ub_ = Vector::Zero(nactuated);
      dq_max_ = Vector::Zero(nactuated);
      ddq_max_ = Vector::Zero(nactuated);

      q0_.resize(ndofs);
      q_.resize(ndofs);
      dq_.resize(ndofs);
      ddq_.resize(ndofs);
      tau_.resize(ndofs);
      q_.setZero(q_.size());
      dq_.setZero(dq_.size());
      ddq_.setZero(ddq_.size());
      tau_.setZero(tau_.size());

      if (!sot_config_path.empty())
        parse_configuration_yaml(sot_config_path);

      mimic_joint_names_ = mimic_joint_names;    
      tsid_joint_names_ = all_dofs(false);
      non_mimic_indexes_ = get_non_mimics_indexes();

      set_stack_configuration();
      init_references();
    }


    std::vector<int> TalosPosTracking::get_non_mimics_indexes()
    {
      std::vector<int> non_mimic_indexes;
      std::vector<int> mimic_indexes;
      if(!mimic_joint_names_.empty())
      {
        for (auto &m : mimic_joint_names_)
        {
          auto it = std::find(tsid_joint_names_.begin(), tsid_joint_names_.end(), m);
          assert(it!=tsid_joint_names_.end());
          mimic_indexes.push_back(std::distance(tsid_joint_names_.begin(), it));
        }
        for (int i = 0; i < q_.size(); i++)
        {
          auto it = std::find(mimic_indexes.begin(), mimic_indexes.end(), i);
          if (it == mimic_indexes.end())
            non_mimic_indexes.push_back(i);
        }
      }
      else
      {
        non_mimic_indexes.resize(q_.size());
        std::generate(non_mimic_indexes.begin(), non_mimic_indexes.end(), [n = 0]() mutable { return n++; });
      }
      return non_mimic_indexes;
    }

    void TalosPosTracking::parse_configuration_yaml(const std::string &sot_config_path)
    {
      std::ifstream yamlConfigFile(sot_config_path);
      if (!yamlConfigFile.good())
      {
        if (verbose_)
        {
          std::cout << sot_config_path << " not found" << std::endl;
          std::cout << "Taking default stack of tasks weights" << std::endl;
        }
      }
      else
      {
        if (verbose_)
          std::cout << "Taking the stack of tasks parameters in " << sot_config_path << std::endl;

        YAML::Node config = YAML::LoadFile(sot_config_path);
        parse(w_com_, "w_com_", config, verbose_);
        parse(w_posture_, "w_posture_", config, verbose_);
        parse(w_forceRef_feet_, "w_forceRef_feet_", config, verbose_);
        parse(w_forceRef_hands_, "w_forceRef_hands_", config, verbose_);
        parse(w_floatingb_, "w_floatingb_", config, verbose_);
        parse(w_velocity_, "w_velocity_", config, verbose_);
        parse(w_rh_, "w_rh_", config, verbose_);
        parse(w_lh_, "w_lh_", config, verbose_);
        parse(w_rf_, "w_rf_", config, verbose_);
        parse(w_lf_, "w_lf_", config, verbose_);
        parse(kp_contact_, "kp_contact_", config, verbose_);
        parse(kp_com_, "kp_com_", config, verbose_);
        parse(kp_posture_, "kp_posture_", config, verbose_);
        parse(kp_floatingb_, "kp_floatingb_", config, verbose_);
        parse(kp_rh_, "kp_rh_", config, verbose_);
        parse(kp_lh_, "kp_lh_", config, verbose_);
        parse(kp_rf_, "kp_rf_", config, verbose_);
        parse(kp_lf_, "kp_lf_", config, verbose_);
      }
    }

    void TalosPosTracking::set_stack_configuration()
    {
      ////////////////////Gather Initial Pose //////////////////////////////////////
      q_tsid_ = robot_->model().referenceConfigurations["pal_start"];
      Quaternion quat = {.w = q_tsid_(6), .x = q_tsid_(3), .y = q_tsid_(4), .z = q_tsid_(5)}; //convert quaternion to euler for dart
      EulerAngles euler = to_euler(quat);
      q0_ << q_tsid_.head(3), euler.roll, euler.pitch, euler.yaw, q_tsid_.tail(robot_->na()); //q_tsid_ is of size 37 (pos+quat+nactuated)

      ////////////////////Create the inverse-dynamics formulation///////////////////
      tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);

      ////////////////////Create an HQP solver /////////////////////////////////////
      solver_ = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG_FAST, "solver-eiquadprog");
      solver_->resize(tsid_->nVar(), tsid_->nEq(), tsid_->nIn());

      ////////////////////Compute Problem Data at init /////////////////////////////
      const uint nv = robot_->nv();
      tsid_->computeProblemData(dt_, q_tsid_, Vector::Zero(nv));
      const pinocchio::Data &data = tsid_->data();

      ////////////////////Compute Tasks, Bounds and Contacts ///////////////////////
      ////////// Add the contacts
      Matrix3x contact_points_(3, 4);
      contact_points_ << -lxn_, -lxn_, +lxp_, +lxp_,
          -lyn_, +lyp_, -lyn_, +lyp_,
          lz_, lz_, lz_, lz_;

      contactRF_ = std::make_shared<Contact6d>("contact_rfoot", *robot_, rf_frame_name_,
                                               contact_points_, contactNormal_,
                                               mu_, fMin_, fMax_);
      contactRF_->Kp(kp_contact_ * Vector::Ones(6));
      contactRF_->Kd(2.0 * contactRF_->Kp().cwiseSqrt());
      contact_rf_ref_ = robot_->position(data, robot_->model().getJointId(rf_frame_name_));
      contactRF_->setReference(contact_rf_ref_);

      tsid_->addRigidContact(*contactRF_, w_forceRef_feet_);
      contactLF_ = std::make_shared<Contact6d>("contact_lfoot", *robot_, lf_frame_name_,
                                               contact_points_, contactNormal_,
                                               mu_, fMin_, fMax_);
      contactLF_->Kp(kp_contact_ * Vector::Ones(6));
      contactLF_->Kd(2.0 * contactLF_->Kp().cwiseSqrt());
      contact_lf_ref_ = robot_->position(data, robot_->model().getJointId(lf_frame_name_));
      contactLF_->setReference(contact_lf_ref_);
      tsid_->addRigidContact(*contactLF_, w_forceRef_feet_);

      ////////// Add the com task
      com_task_ = std::make_shared<TaskComEquality>("task-com", *robot_);
      com_task_->Kp(kp_com_ * Vector::Ones(3));
      com_task_->Kd(2.0 * com_task_->Kp().cwiseSqrt());
      tsid_->addMotionTask(*com_task_, w_com_, 1);

      ////////// Add the posture task
      posture_task_ = std::make_shared<TaskJointPosture>("task-posture", *robot_);
      posture_task_->Kp(kp_posture_ * Vector::Ones(nv - 6));
      posture_task_->Kd(2.0 * posture_task_->Kp().cwiseSqrt());
      Vector mask_post = Vector::Ones(nv - 6);
      // for(int i =0; i < 11; i++){
      //   mask_post[i] = 0;
      // }
      posture_task_->setMask(mask_post);
      tsid_->addMotionTask(*posture_task_, w_posture_, 1);

      ////////// Add the floatingb task
      floatingb_task_ = std::make_shared<TaskSE3Equality>("task-floatingb", *robot_, fb_joint_name_);
      floatingb_task_->Kp(kp_floatingb_ * Vector::Ones(6));
      floatingb_task_->Kd(2.0 * floatingb_task_->Kp().cwiseSqrt());
      Vector mask_floatingb = Vector::Ones(6);
      for (int i = 0; i < 3; i++)
      {
        mask_floatingb[i] = 0; // DO NOT CONSTRAIN floatingb POSITION IN SOT
      }
      floatingb_task_->setMask(mask_floatingb);
      tsid_->addMotionTask(*floatingb_task_, w_floatingb_, 1);

      ////////// Add the left hand  task
      lh_task_ = std::make_shared<TaskSE3Equality>("task-lh", *robot_, "gripper_left_joint");
      lh_task_->Kp(kp_lh_ * Vector::Ones(6));
      lh_task_->Kd(2.0 * lh_task_->Kp().cwiseSqrt());
      Vector mask_lh = Vector::Ones(6);
      for (int i = 3; i < 6; i++)
      {
        mask_lh[i] = 0; // DO NOT CONSTRAIN HAND ORIENTATION IN SOT
      }
      lh_task_->setMask(mask_lh);
      tsid_->addMotionTask(*lh_task_, w_lh_, 1);

      ////////// Add the right hand task
      rh_task_ = std::make_shared<TaskSE3Equality>("task-rh", *robot_, "gripper_right_joint");
      rh_task_->Kp(kp_rh_ * Vector::Ones(6));
      rh_task_->Kd(2.0 * rh_task_->Kp().cwiseSqrt());
      Vector mask_rh = Vector::Ones(6);
      for (int i = 3; i < 6; i++)
      {
        mask_rh[i] = 0; // DO NOT CONSTRAIN HAND ORIENTATION IN SOT
      }
      rh_task_->setMask(mask_rh);
      tsid_->addMotionTask(*rh_task_, w_rh_, 1);

      ////////// Add the left foot  task
      lf_task_ = std::make_shared<TaskSE3Equality>("task-lf", *robot_, "leg_left_6_joint");
      lf_task_->Kp(kp_lf_ * Vector::Ones(6));
      lf_task_->Kd(2.0 * lf_task_->Kp().cwiseSqrt());
      Vector maskLf = Vector::Ones(6);
      lf_task_->setMask(maskLf);
      tsid_->addMotionTask(*lf_task_, w_lf_, 1);

      ////////// Add the right foot  task
      rf_task_ = std::make_shared<TaskSE3Equality>("task-rf", *robot_, "leg_right_6_joint");
      rf_task_->Kp(kp_rf_ * Vector::Ones(6));
      rf_task_->Kd(2.0 * rf_task_->Kp().cwiseSqrt());
      Vector maskRf = Vector::Ones(6);
      rf_task_->setMask(maskRf);
      tsid_->addMotionTask(*rf_task_, w_rf_, 1);

      ////////// Add the position, velocity and acceleration limits
      bounds_task_ = std::make_shared<TaskJointPosVelAccBounds>("task-posVelAcc-bounds", *robot_, dt_, verbose_);
      dq_max_ = robot_->model().velocityLimit.tail(robot_->na());
      ddq_max_ = dq_max_ / dt_;
      bounds_task_->setVelocityBounds(dq_max_);
      bounds_task_->setAccelerationBounds(ddq_max_);
      q_lb_ = robot_->model().lowerPositionLimit.tail(robot_->na());
      q_ub_ = robot_->model().upperPositionLimit.tail(robot_->na());
      bounds_task_->setPositionBounds(q_lb_, q_ub_);
      tsid_->addMotionTask(*bounds_task_, w_velocity_, 0); //add pos vel acc bounds
    }

    void TalosPosTracking::init_references()
    {
      com_init_ = robot_->com(tsid_->data());
      posture_init_ = q_tsid_.tail(robot_->na());
      floatingb_init_ = robot_->position(tsid_->data(), robot_->model().getJointId(fb_joint_name_));
      lf_init_ = robot_->position(tsid_->data(), robot_->model().getJointId("leg_left_6_joint"));
      rf_init_ = robot_->position(tsid_->data(), robot_->model().getJointId("leg_right_6_joint"));
      lh_init_ = robot_->position(tsid_->data(), robot_->model().getJointId("gripper_left_joint"));
      rh_init_ = robot_->position(tsid_->data(), robot_->model().getJointId("gripper_right_joint"));

      com_ref_ = com_init_;
      posture_ref_ = posture_init_;
      floatingb_ref_ = floatingb_init_;
      lf_ref_ = lf_init_;
      rf_ref_ = rf_init_;
      lh_ref_ = lh_init_;
      rh_ref_ = rh_init_;

      traj_com_ = std::make_shared<TrajectoryEuclidianConstant>("traj_com", com_ref_);
      traj_posture_ = std::make_shared<TrajectoryEuclidianConstant>("traj_posture", posture_ref_);
      traj_floatingb_ = std::make_shared<TrajectorySE3Constant>("traj_floatingb", floatingb_ref_);
      traj_lf_ = std::make_shared<TrajectorySE3Constant>("traj_lf", lf_ref_);
      traj_rf_ = std::make_shared<TrajectorySE3Constant>("traj_rf", rf_ref_);
      traj_lh_ = std::make_shared<TrajectorySE3Constant>("traj_lh", lh_ref_);
      traj_rh_ = std::make_shared<TrajectorySE3Constant>("traj_rh", rh_ref_);

      TrajectorySample sample_com_ = traj_com_->computeNext();
      TrajectorySample sample_posture_ = traj_posture_->computeNext();
      TrajectorySample sample_floatingb_ = traj_floatingb_->computeNext();
      TrajectorySample sample_lf_ = traj_lf_->computeNext();
      TrajectorySample sample_rf_ = traj_rf_->computeNext();
      TrajectorySample sample_lh_ = traj_lh_->computeNext();
      TrajectorySample sample_rh_ = traj_rh_->computeNext();

      com_task_->setReference(sample_com_);
      posture_task_->setReference(sample_posture_);
      floatingb_task_->setReference(sample_floatingb_);
      lf_task_->setReference(sample_lf_);
      rf_task_->setReference(sample_rf_);
      lh_task_->setReference(sample_lh_);
      rh_task_->setReference(sample_rh_);

      set_task_traj_map();
    }

    bool TalosPosTracking::solve()
    {
      //Compute the current data from the current position and solve to find next position
      const HQPData &HQPData = tsid_->computeProblemData(t_, q_tsid_, v_tsid_);
      const HQPOutput &sol = solver_->solve(HQPData);
      if (sol.status == HQP_STATUS_OPTIMAL)
      {
        const Vector &tau = tsid_->getActuatorForces(sol);
        const Vector &dv = tsid_->getAccelerations(sol);
        tau_tsid_ = tau;
        a_tsid_ = dv;
        v_tsid_ += dt_ * dv;
        q_tsid_ = pinocchio::integrate(robot_->model(), q_tsid_, dt_ * v_tsid_);
        t_ += dt_;

        //Convert quaternion to euler to form q_ for dart
        Quaternion quat = {.w = q_tsid_(6), .x = q_tsid_(3), .y = q_tsid_(4), .z = q_tsid_(5)};
        EulerAngles euler = to_euler(quat);
        q_ << q_tsid_.head(3), euler.roll, euler.pitch, euler.yaw, q_tsid_.tail(robot_->nq() - 7); //q_tsid_ of size 37 (pos+quat+nactuated)
        dq_ = v_tsid_;                                                                             //the speed of the free flyerjoint is dim 6 even if its pos id dim 7
        tau_ << 0, 0, 0, 0, 0, 0, tau_tsid_;                                                       //the size of tau is actually 30 (nactuated)
        ddq_ = a_tsid_;
        return true;
      }
      else
      {
        if (verbose_)
        {
          std::cout << "Controller failed, can't solve problem " << std::endl;
          std::cout << "Status " + toString(sol.status) << std::endl;
        }
        return false;
      }
    }

    // Removes the universe and root (floating base) joint names
    std::vector<std::string> TalosPosTracking::controllable_dofs(bool filter_mimics)
    {
      auto na = robot_->model().names;
      auto tsid_controllables = std::vector<std::string>(robot_->model().names.begin() + 2, robot_->model().names.end());
      if(filter_mimics)
        return remove_intersection(tsid_controllables, mimic_joint_names_);
      else
        return tsid_controllables;
    }

    // Order of the floating base in q_ according to dart naming convention
    std::vector<std::string> TalosPosTracking::floating_base_dofs()
    {
      std::vector<std::string> floating_base_dofs;
      if(fb_joint_name_=="root_joint")
      {
        floating_base_dofs = {"rootJoint_pos_x",
                              "rootJoint_pos_y",
                              "rootJoint_pos_z",
                              "rootJoint_rot_x",
                              "rootJoint_rot_y",
                              "rootJoint_rot_z"};

      }
      else{
        floating_base_dofs = {fb_joint_name_+"_pos_x",
                              fb_joint_name_+"_pos_y",
                              fb_joint_name_+"_pos_z",
                              fb_joint_name_+"_rot_x",
                              fb_joint_name_+"_rot_y",
                              fb_joint_name_+"_rot_z"};

      }
      
      return floating_base_dofs;
    }

    std::vector<std::string> TalosPosTracking::all_dofs(bool filter_mimics)
    {
      std::vector<std::string> all_dofs = floating_base_dofs();
      std::vector<std::string> control_dofs = controllable_dofs(filter_mimics);
      all_dofs.insert(all_dofs.end(), control_dofs.begin(), control_dofs.end());
      if(filter_mimics)
        return remove_intersection(all_dofs, mimic_joint_names_);
      else
        return all_dofs;
    }

    //Left hand trajectory control
    void TalosPosTracking::add_to_lh_ref(float delta_x, float delta_y, float delta_z)
    {
      lh_ref_.translation()(0) += delta_x;
      lh_ref_.translation()(1) += delta_y;
      lh_ref_.translation()(2) += delta_z;
      traj_lh_->setReference(lh_ref_);
      TrajectorySample sample_lh_ = traj_lh_->computeNext();
      lh_task_->setReference(sample_lh_);
    }

    void TalosPosTracking::set_com_ref(const Vector3& ref)
    {
      traj_com_->setReference(ref);
      TrajectorySample sample_com_ = traj_com_->computeNext();
      com_task_->setReference(sample_com_);
    }

    Eigen::VectorXd TalosPosTracking::dq(bool filter_mimics)
    {
      if(filter_mimics)
        return slice_vec(dq_,non_mimic_indexes_);
      else
        return dq_;
    }

    Eigen::VectorXd TalosPosTracking::q(bool filter_mimics)
    {
      if(filter_mimics)
        return slice_vec(q_,non_mimic_indexes_);
      else
        return q_;
    }

    Eigen::VectorXd TalosPosTracking::q0(bool filter_mimics)
    {
      if(filter_mimics)
        return slice_vec(q0_,non_mimic_indexes_);
      else
        return q0_;
    }

    Eigen::VectorXd TalosPosTracking::filter_cmd(const Eigen::VectorXd& cmd)
    {
      return slice_vec(cmd,non_mimic_indexes_);
    }

    void TalosPosTracking::set_task_traj_map()
    {
      TaskTrajReferenceVector3 com = {.task = com_task_, .ref = com_init_, .traj = traj_com_};
      task_traj_map_["com"] = com;
      TaskTrajReferenceSE3 lh = {.task = lh_task_, .ref = lh_init_, .traj = traj_lh_};
      task_traj_map_["lh"] = lh;
      TaskTrajReferenceSE3 rh = {.task = rh_task_, .ref = rh_init_, .traj = traj_rh_};
      task_traj_map_["rh"] = rh;
    }



} // namespace tsid_sot
