#ifndef TALOS_BASE_CONTROLLER_HPP
#define TALOS_BASE_CONTROLLER_HPP

#include <Eigen/Core>
#include <string>
#include <unordered_map>
#include <iostream>
#include "boost/variant.hpp"
#include "string"
#include "utils/utils.hpp"

#include <tsid/math/fwd.hpp>
#include <tsid/trajectories/trajectory-base.hpp>
#include <tsid/trajectories/trajectory-se3.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-se3-equality.hpp>

// forward declaration to speed-up compilation
namespace tsid
{
    class InverseDynamicsFormulationAccForce;
    namespace trajectories
    {
        class TrajectoryBase;
        class TrajectorySample;
        class TrajectorySE3Constant;
        class TrajectoryEuclidianConstant;
    } // namespace trajectories
    namespace contacts
    {
        class Contact6d;
    }

    namespace solvers
    {
        class SolverHQPBase;
    }
    namespace robots
    {
        class RobotWrapper;
    }
    namespace tasks
    {
        class TaskComEquality;
        class TaskJointPosture;
        class TaskSE3Equality;
        class TaskJointPosVelAccBounds;
    } // namespace tasks
} // namespace tsid

namespace tsid_sot
{
    namespace controllers
    {
        class TalosBaseController
        {
        public:
            struct Params
            {
                std::string urdf_path;
                std::string srdf_path;
                std::string sot_config_path;
                std::string floating_base_joint_name;
                float dt;
                bool verbose;
                std::vector<std::string> mimic_dof_names;
            };

            template <typename TaskType, typename ReferenceType, typename TrajType>
            struct TaskTrajReference
            {
                std::shared_ptr<TaskType> task;
                ReferenceType ref;
                std::shared_ptr<TrajType> traj;
            };
            using TaskTrajReferenceSE3 = TaskTrajReference<tsid::tasks::TaskSE3Equality, pinocchio::SE3, tsid::trajectories::TrajectorySE3Constant>;
            using TaskTrajReferenceVector3 = TaskTrajReference<tsid::tasks::TaskComEquality, tsid::math::Vector3, tsid::trajectories::TrajectoryEuclidianConstant>;
            // typedef boost::variant<pinocchio::SE3, tsid::math::Vector3> ReferenceType;

            TalosBaseController(const Params &params);

            ~TalosBaseController(){};

            bool solve();

            // Removes the universe and root (floating base) joint names
            std::vector<std::string> controllable_dofs(bool filter_mimics = true);
            // Order of the floating base in q_ according to dart naming convention
            std::vector<std::string> floating_base_dofs();
            std::vector<std::string> all_dofs(bool filter_mimics = true);

            std::vector<int> non_mimic_indexes() { return non_mimic_indexes_; }
            Eigen::VectorXd filter_cmd(const Eigen::VectorXd &cmd);

            Eigen::VectorXd dq(bool filter_mimics = true);
            Eigen::VectorXd q0(bool filter_mimics = true);
            Eigen::VectorXd q(bool filter_mimics = true);

        private:
            std::vector<int> get_non_mimics_indexes();
            virtual void parse_configuration_yaml(const std::string &sot_config_path) = 0;
            virtual void set_stack_configuration() = 0;
            virtual void init_references() = 0;
            virtual void set_task_traj_map() = 0;

        protected:
            bool verbose_;
            double t_;
            double dt_;

            std::string fb_joint_name_; //name of the floating base joint
            std::vector<std::string> mimic_dof_names_;
            std::vector<std::string> tsid_joint_names_; //contain floating base and mimics
            std::vector<int> non_mimic_indexes_;

            tsid::math::Vector q_tsid_;   // tsid joint positions
            tsid::math::Vector v_tsid_;   // tsid joint velocities
            tsid::math::Vector a_tsid_;   // tsid joint accelerations
            tsid::math::Vector tau_tsid_; // tsid joint torques
            Eigen::VectorXd q0_;          // tsid joint positions resized for dart
            Eigen::VectorXd q_;           // tsid joint positions resized for dart
            Eigen::VectorXd dq_;          // tsid joint velocities resized for dart
            Eigen::VectorXd ddq_;         // tsid joint accelerations resized for dart
            Eigen::VectorXd tau_;         // tsid joint torques resized for dart

            std::shared_ptr<tsid::robots::RobotWrapper> robot_;
            std::shared_ptr<tsid::InverseDynamicsFormulationAccForce> tsid_;
            tsid::solvers::SolverHQPBase *solver_;

            // limits (position, velocity, acceleration)
            tsid::math::Vector q_lb_;    // lower position bound
            tsid::math::Vector q_ub_;    // upper position bound
            tsid::math::Vector dq_max_;  // max velocity bound
            tsid::math::Vector ddq_max_; // max acceleration bound

            std::unordered_map<std::string, boost::variant<TaskTrajReferenceSE3, TaskTrajReferenceVector3>> task_traj_map_;
        };

        tsid_sot::controllers::TalosBaseController::Params parse_params(YAML::Node config);
    } // namespace controllers
} // namespace tsid_sot
#endif
