#ifndef IWBC_TALOS_BASE_CONTROLLER_HPP
#define IWBC_TALOS_BASE_CONTROLLER_HPP

#include <Eigen/Core>
#include <boost/variant.hpp>
#include <iostream>
#include <string>
#include <unordered_map>

#include <pinocchio/spatial/se3.hpp>

#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/math/fwd.hpp>
#include <tsid/robots/fwd.hpp>
#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/trajectories/trajectory-base.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>
#include <tsid/trajectories/trajectory-se3.hpp>

#include <inria_wbc/utils/utils.hpp>

// forward declaration to speed-up compilation
namespace tsid {
    class InverseDynamicsFormulationAccForce;
    namespace trajectories {
        class TrajectoryBase;
        class TrajectorySample;
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

namespace inria_wbc {
    namespace controllers {
        class TalosBaseController {
        public:
            using opt_params_t = std::map<std::string, double>;
            struct Params {
                std::string urdf_path;
                std::string srdf_path;
                std::string sot_config_path;
                std::string floating_base_joint_name;
                float dt;
                bool verbose;
                std::vector<std::string> mimic_dof_names;
                opt_params_t opt_params; // parameters that can be optimized
            };

            template <typename TaskType, typename ReferenceType, typename TrajType>
            struct TaskTrajReference {
                std::shared_ptr<TaskType> task;
                ReferenceType ref;
                std::shared_ptr<TrajType> traj;
            };
            using TaskTrajReferenceSE3 = TaskTrajReference<tsid::tasks::TaskSE3Equality, pinocchio::SE3, tsid::trajectories::TrajectorySE3Constant>;
            using TaskTrajReferenceVector3 = TaskTrajReference<tsid::tasks::TaskComEquality, tsid::math::Vector3, tsid::trajectories::TrajectoryEuclidianConstant>;

            TalosBaseController(const Params& params);
            TalosBaseController(const TalosBaseController&) = delete;
            TalosBaseController& operator=(const TalosBaseController& o) = delete;
            virtual ~TalosBaseController(){};

            bool solve();

            // Removes the universe and root (floating base) joint names
            std::vector<std::string> controllable_dofs(bool filter_mimics = true) const;
            // Order of the floating base in q_ according to dart naming convention
            std::vector<std::string> floating_base_dofs() const;
            std::vector<std::string> all_dofs(bool filter_mimics = true) const;
            const tsid::math::Vector3& get_pinocchio_com() const { return robot_->com(tsid_->data()); }

            const std::vector<int>& non_mimic_indexes() const { return non_mimic_indexes_; }
            Eigen::VectorXd filter_cmd(const Eigen::VectorXd& cmd) const { return utils::slice_vec(cmd, non_mimic_indexes_); }

            Eigen::VectorXd ddq(bool filter_mimics = true) const;
            Eigen::VectorXd dq(bool filter_mimics = true) const;
            Eigen::VectorXd q0(bool filter_mimics = true) const;
            Eigen::VectorXd q(bool filter_mimics = true) const;

            double dt() const { return dt_; };
            const Params& params() const { return params_; };

            std::shared_ptr<tsid::robots::RobotWrapper> robot() { return robot_; };
            std::shared_ptr<tsid::InverseDynamicsFormulationAccForce> tsid() { return tsid_; };
            std::vector<double> pinocchio_model_masses() const;
            const std::vector<double>& pinocchio_model_cumulated_masses() { return tsid_->data().mass; };
            const std::vector<std::string>& pinocchio_joint_names() const { return robot_->model().names; }

            // parameters that can be optimized / tuned online
            // (e.g., weights of task, gains of the tasks, etc.)
            virtual const opt_params_t& opt_params() const
            {
                assert(0 && "calling opt_params but no param to set in base controller");
                static opt_params_t x;
                return x;
            }

        private:
            std::vector<int> get_non_mimics_indexes() const;

        protected:
            void _reset();

            Params params_;
            bool verbose_;
            double t_;
            double dt_;

            std::string fb_joint_name_; //name of the floating base joint
            std::vector<std::string> mimic_dof_names_;
            std::vector<std::string> tsid_joint_names_; //contain floating base and mimics
            std::vector<int> non_mimic_indexes_;

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
            std::shared_ptr<tsid::solvers::SolverHQPBase> solver_;

            // limits (position, velocity, acceleration)
            tsid::math::Vector q_lb_; // lower position bound
            tsid::math::Vector q_ub_; // upper position bound
            tsid::math::Vector dq_max_; // max velocity bound
            tsid::math::Vector ddq_max_; // max acceleration bound

            std::unordered_map<std::string, boost::variant<TaskTrajReferenceSE3, TaskTrajReferenceVector3>> task_traj_map_;
        };

        inria_wbc::controllers::TalosBaseController::Params parse_params(YAML::Node config);
    } // namespace controllers
} // namespace inria_wbc
#endif
