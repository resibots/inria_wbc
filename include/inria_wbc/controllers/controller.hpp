#ifndef IWBC_TALOS_BASE_CONTROLLER_HPP
#define IWBC_TALOS_BASE_CONTROLLER_HPP

#include <Eigen/Core>
#include <boost/variant.hpp>
#include <iostream>
#include <string>
#include <unordered_map>

#include <pinocchio/spatial/se3.hpp>

#include <tsid/contacts/contact-6d.hpp>
#include <tsid/contacts/contact-point.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/math/fwd.hpp>
#include <tsid/math/utils.hpp>
#include <tsid/robots/fwd.hpp>
#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/tasks/task-actuation-bounds.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-joint-bounds.hpp>
#include <tsid/tasks/task-joint-posVelAcc-bounds.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/trajectories/trajectory-base.hpp>

#include <inria_wbc/utils/factory.hpp>
#include <inria_wbc/utils/utils.hpp>

namespace inria_wbc {

    namespace se3_mask
    {
        using mask6 = Eigen::Array<double, 6, 1>;

        static const mask6 all = (mask6() << 1, 1, 1, 1, 1, 1).finished();
        static const mask6 xyz = (mask6() << 1, 1, 1, 0, 0, 0).finished();
        static const mask6 rpy = (mask6() << 0, 0, 0, 1, 1, 1).finished();
        static const mask6 x   = (mask6() << 1, 0, 0, 0, 0, 0).finished();
        static const mask6 y   = (mask6() << 0, 1, 0, 0, 0, 0).finished();
        static const mask6 z   = (mask6() << 0, 0, 1, 0, 0, 0).finished();
        static const mask6 roll  = (mask6() << 0, 0, 0, 1, 0, 0).finished();
        static const mask6 pitch = (mask6() << 0, 0, 0, 0, 1, 0).finished();
        static const mask6 yaw   = (mask6() << 0, 0, 0, 0, 0, 1).finished();
    }

    namespace controllers {
        class Controller {
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

            Controller(const Params& params);
            Controller(const Controller&) = delete;
            Controller& operator=(const Controller& o) = delete;
            virtual ~Controller(){};

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

            std::shared_ptr<tsid::tasks::TaskComEquality> make_com_task(const std::string& name, double kp) const;
            std::shared_ptr<tsid::tasks::TaskJointPosture> make_posture_task(const std::string& name, double kp) const;
            std::shared_ptr<tsid::tasks::TaskSE3Equality> make_se3_frame_task(const std::string& name, const std::string& frame_name, double kp, const se3_mask::mask6& mask = se3_mask::all) const;
            std::shared_ptr<tsid::tasks::TaskSE3Equality> make_se3_joint_task(const std::string& name, const std::string& joint_name, double kp, const se3_mask::mask6& mask = se3_mask::all) const;
            std::shared_ptr<tsid::tasks::TaskJointPosVelAccBounds> make_bound_task(const std::string& name) const;

            Params params_;
            bool verbose_;
            double t_;
            double dt_;

            std::string fb_joint_name_; //name of the floating base joint
            std::vector<std::string> mimic_dof_names_;
            std::vector<std::string> tsid_joint_names_; //contain floating base and mimics
            std::vector<int> non_mimic_indexes_;

            //---- TSID conventions for the floating base: quaternion
            tsid::math::Vector q_tsid_; // tsid joint positions
            tsid::math::Vector v_tsid_; // tsid joint velocities
            tsid::math::Vector a_tsid_; // tsid joint accelerations
            tsid::math::Vector tau_tsid_; // tsid joint torques

            //---- Dart conventions for the floating base: axis-angle
            Eigen::VectorXd q0_; // tsid joint positions resized for dart
            Eigen::VectorXd q_; // tsid joint positions resized for dart
            Eigen::VectorXd dq_; // tsid joint velocities resized for dart
            Eigen::VectorXd ddq_; // tsid joint accelerations resized for dart
            Eigen::VectorXd tau_; // tsid joint torques resized for dart

            std::shared_ptr<tsid::robots::RobotWrapper> robot_;
            std::shared_ptr<tsid::InverseDynamicsFormulationAccForce> tsid_;
            std::shared_ptr<tsid::solvers::SolverHQPBase> solver_;
        };

        Controller::Params parse_params(YAML::Node config);

        inline tsid::trajectories::TrajectorySample to_sample(const Eigen::VectorXd& ref)
        {
            tsid::trajectories::TrajectorySample sample;
            sample.pos = ref;
            sample.vel.setZero(ref.size());
            sample.acc.setZero(ref.size());
            return sample;
        }

        inline tsid::trajectories::TrajectorySample to_sample(const pinocchio::SE3& ref)
        {
            tsid::trajectories::TrajectorySample sample;
            sample.resize(12, 6);
            tsid::math::SE3ToVector(ref, sample.pos);
            return sample;
        }

        using Factory = utils::Factory<Controller, Controller::Params>;
        template <typename T>
        struct Register : public utils::AutoRegister<Controller, T, Controller::Params> {
            Register(const std::string& name) : utils::AutoRegister<Controller, T, Controller::Params>(name) {}
        };
    } // namespace controllers
} // namespace inria_wbc
#endif
