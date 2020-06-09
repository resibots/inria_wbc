#ifndef TSID_SOT_TALOS_HPP
#define TSID_SOT_TALOS_HPP

#include <Eigen/Core>
#include <string>

#include <tsid/math/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
namespace tsid {
    class InverseDynamicsFormulationAccForce;
    namespace trajectories {
        class TrajectoryBase;
        class TrajectorySE3Constant;
        class TrajectoryEuclidianConstant;
    } // namespace trajectories
    namespace contacts {
        class Contact6d;
    }

    namespace solvers {
        class SolverHQPBase;
    }
    namespace robots {
      class RobotWrapper;
    }
    namespace tasks {
        class TaskComEquality;
        class TaskJointPosture;
        class TaskSE3Equality;
        class TaskJointPosVelAccBounds;
    } // namespace tasks
} // namespace tsid

namespace tsid_sot {
    class Talos {
    public:
        struct Params {
            std::string urdf_path;
            std::string srdf_path;
            float dt;
        };

        Talos(const Params& params, const std::string& sot_config_path = "", bool verbose = true);
        ~Talos(){};

        void parse_configuration_yaml(const std::string& sot_config_path);
        void set_stack_configuration();
        void init_references();
        bool solve();

        // Removes the universe and root (floating base) joint names
        std::vector<std::string> controllable_dofs();

        // Order of the floating base in q_ according to dart naming convention
        std::vector<std::string> floating_base_dofs();
        std::vector<std::string> all_dofs();

        // Left hand trajectory control
        void add_to_lh_ref(float delta_x, float delta_y, float delta_z);

        pinocchio::SE3 lh_init() { return lh_init_; }
        Eigen::VectorXd dq() { return dq_; }
        Eigen::VectorXd q0() { return q0_; }
        Eigen::VectorXd q() { return q_; }
        // COM trajectory control
        tsid::math::Vector3 com_init() { return com_init_; }

        std::shared_ptr<tsid::tasks::TaskComEquality> com_task() { return com_task_; }

        void set_com_ref(const tsid::math::Vector3& ref);

    private:
        // TALOS CONFIG
        double lxp_ = 0.1; // foot length in positive x direction
        double lxn_ = 0.11; // foot length in negative x direction
        double lyp_ = 0.069; // foot length in positive y direction
        double lyn_ = 0.069; // foot length in negative y direction
        double lz_ = 0.107; // foot sole height with respect to ankle joint
        double mu_ = 0.3; // friction coefficient
        double fMin_ = 5.0; // minimum normal force
        double fMax_ = 1500.0; // maximum normal force
        std::string rf_frame_name_ = "leg_right_6_joint"; // right foot joint name
        std::string lf_frame_name_ = "leg_left_6_joint"; // left foot joint name
        tsid::math::Vector3 contactNormal_
            = tsid::math::Vector3::UnitZ(); // direction of the normal to the contact surface
        double w_com_ = 1.0; //  weight of center of mass task
        double w_posture_ = 0.75; //  weight of joint posture task
        double w_forceRef_feet_ = 1e-3; //# weight of force regularization task
        double w_forceRef_hands_ = 1e-3; //# weight of force regularization task
        double w_floatingb_ = 20.0; //# weight of floatingb task
        double w_velocity_ = 1.0; //# weight of velocity bounds
        double w_rh_ = 10.0; //# weight of right hand  task
        double w_lh_ = 10.0; //# weight of left hand  task
        double w_rf_ = 1.0; //# weight of right foot  task
        double w_lf_ = 1.0; //# weight of left foot  task
        double kp_contact_ = 30.0; //# proportional gain of contact constraint
        double kp_com_ = 3000.0; //# proportional gain of center of mass task
        double kp_posture_ = 30.0; //# proportional gain of joint posture task
        double kp_floatingb_ = 3000.0; //# proportional gain of floatingb task
        double kp_rh_ = 300.0; //# proportional gain of right hand task
        double kp_lh_ = 300.0; //# proportional gain of left hand task
        double kp_rf_ = 30.0; //# proportional gain of right foot task
        double kp_lf_ = 30.0; //# proportional gain of left foot task
        double dt_;
        bool verbose_;

        double t_;

        tsid::math::Vector q_tsid_; // tsid joint positions
        tsid::math::Vector v_tsid_; // tsid joint velocities
        tsid::math::Vector a_tsid_; // tsid joint accelerations
        tsid::math::Vector tau_tsid_; // tsid joint torques
        Eigen::VectorXd q0_; // tsid joint positions resized for dart
        Eigen::VectorXd q_; // tsid joint positions resized for dart
        Eigen::VectorXd dq_; // tsid joint velocities resized for dart
        Eigen::VectorXd ddq_; // tsid joint accelerations resized for dart
        Eigen::VectorXd tau_; // tsid joint torques resized for dart

        std::shared_ptr<tsid::robots::RobotWrapper> robot_;
        std::shared_ptr<tsid::InverseDynamicsFormulationAccForce> tsid_;
        tsid::solvers::SolverHQPBase* solver_;

        // contacts
        tsid::math::Matrix3x contact_points_;
        pinocchio::SE3 contact_rf_ref_, contact_lf_ref_;
        std::shared_ptr<tsid::contacts::Contact6d> contactRF_;
        std::shared_ptr<tsid::contacts::Contact6d> contactLF_;

        // tasks
        std::shared_ptr<tsid::tasks::TaskComEquality> com_task_;
        std::shared_ptr<tsid::tasks::TaskJointPosture> posture_task_;
        std::shared_ptr<tsid::tasks::TaskSE3Equality> floatingb_task_;
        std::shared_ptr<tsid::tasks::TaskSE3Equality> lf_task_;
        std::shared_ptr<tsid::tasks::TaskSE3Equality> rf_task_;
        std::shared_ptr<tsid::tasks::TaskSE3Equality> lh_task_;
        std::shared_ptr<tsid::tasks::TaskSE3Equality> rh_task_;
        std::shared_ptr<tsid::tasks::TaskJointPosVelAccBounds> bounds_task_;

        // limits (position, velocity, acceleration)
        tsid::math::Vector q_lb_; // lower position bound
        tsid::math::Vector q_ub_; // upper position bound
        tsid::math::Vector dq_max_; // max velocity bound
        tsid::math::Vector ddq_max_; // max acceleration bound

        // com ref
        tsid::math::Vector3 com_init_, com_ref_;
        std::shared_ptr<tsid::trajectories::TrajectoryEuclidianConstant> traj_com_;

        // posture ref
        tsid::math::Vector posture_init_, posture_ref_;
        std::shared_ptr<tsid::trajectories::TrajectoryBase> traj_posture_;

        // floatingb ref
        pinocchio::SE3 floatingb_init_, floatingb_ref_;
        std::shared_ptr<tsid::trajectories::TrajectorySE3Constant> traj_floatingb_;

        // Left Foot ref
        pinocchio::SE3 lf_init_, lf_ref_;
        std::shared_ptr<tsid::trajectories::TrajectorySE3Constant> traj_lf_;

        // Right Foot ref
        pinocchio::SE3 rf_init_, rf_ref_;
        std::shared_ptr<tsid::trajectories::TrajectorySE3Constant> traj_rf_;

        // Left Hand ref
        pinocchio::SE3 lh_init_, lh_ref_;
        std::shared_ptr<tsid::trajectories::TrajectorySE3Constant> traj_lh_;

        // Right Hand ref
        pinocchio::SE3 rh_init_, rh_ref_;
        std::shared_ptr<tsid::trajectories::TrajectorySE3Constant> traj_rh_;
    };
} // namespace tsid_sot
#endif
