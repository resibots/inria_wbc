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

    namespace se3_mask {
        using mask6 = Eigen::Array<double, 6, 1>;

        static const mask6 all = (mask6() << 1, 1, 1, 1, 1, 1).finished();
        static const mask6 xyz = (mask6() << 1, 1, 1, 0, 0, 0).finished();
        static const mask6 rpy = (mask6() << 0, 0, 0, 1, 1, 1).finished();
        static const mask6 x = (mask6() << 1, 0, 0, 0, 0, 0).finished();
        static const mask6 y = (mask6() << 0, 1, 0, 0, 0, 0).finished();
        static const mask6 z = (mask6() << 0, 0, 1, 0, 0, 0).finished();
        static const mask6 roll = (mask6() << 0, 0, 0, 1, 0, 0).finished();
        static const mask6 pitch = (mask6() << 0, 0, 0, 0, 1, 0).finished();
        static const mask6 yaw = (mask6() << 0, 0, 0, 0, 0, 1).finished();
    } // namespace se3_mask
    namespace cartesian_mask {
        using mask6 = Eigen::Array<double, 3, 1>;
        static const mask6 xyz = (mask6() << 1, 1, 1).finished();
        static const mask6 x = (mask6() << 1, 0, 0).finished();
        static const mask6 y = (mask6() << 0, 1, 0).finished();
        static const mask6 z = (mask6() << 0, 0, 1).finished();
    } // namespace cartesian_mask

    namespace controllers {

        struct SensorDataStruct {
            typedef std::shared_ptr<SensorDataStruct> Ptr;

            ~SensorDataStruct() { std::cerr << "calling SensorDataStruct destructor"; };
        };

        struct SensorData
        {
            SensorDataStruct::Ptr data_struct_ptr;

            template <class DerivedSensorDataStruct, class... Args>
            static SensorData create(Args... args)
            {
                static_assert(std::is_base_of<SensorDataStruct, DerivedSensorDataStruct>::value, "T should inherit from B");
                return SensorData { .data_struct_ptr = std::make_shared<DerivedSensorDataStruct>(args...) };
            }

            template <class DerivedSensorDataStruct>
            std::shared_ptr<DerivedSensorDataStruct> as() const
            {
                static_assert(std::is_base_of<SensorDataStruct, DerivedSensorDataStruct>::value, "T should inherit from B");
                return std::static_pointer_cast<DerivedSensorDataStruct>(this->data_struct_ptr);
            }

        };

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

            virtual void update(const SensorData& sensor_data) { _solve(); };

            // Removes the universe and root (floating base) joint names
            std::vector<std::string> controllable_dofs(bool filter_mimics = true) const;
            // Order of the floating base in q_ according to dart naming convention
            std::vector<std::string> floating_base_dofs() const;
            std::vector<std::string> all_dofs(bool filter_mimics = true) const;

            virtual const Eigen::Vector2d& cop() const
            {
                static Eigen::Vector2d tmp;
                IWBC_ERROR("No COP estimator in controller.");
                return tmp;
            }
            // this could call a CoM estimator
            virtual const tsid::math::Vector3& com() const { return robot_->com(tsid_->data()); }

            const std::vector<int>& non_mimic_indexes() const { return non_mimic_indexes_; }
            Eigen::VectorXd filter_cmd(const Eigen::VectorXd& cmd) const { return utils::slice_vec(cmd, non_mimic_indexes_); }

            Eigen::VectorXd ddq(bool filter_mimics = true) const;
            Eigen::VectorXd dq(bool filter_mimics = true) const;
            Eigen::VectorXd q0(bool filter_mimics = true) const;
            Eigen::VectorXd q(bool filter_mimics = true) const;
            tsid::math::Vector q_tsid() const { return q_tsid_; };

            double dt() const { return dt_; };
            const Params& params() const { return params_; };

            std::shared_ptr<tsid::robots::RobotWrapper> robot() { return robot_; };
            std::shared_ptr<tsid::InverseDynamicsFormulationAccForce> tsid() { return tsid_; };
            std::vector<double> pinocchio_model_masses() const;
            const std::vector<double>& pinocchio_model_cumulated_masses() { return tsid_->data().mass; };
            const std::vector<std::string>& pinocchio_joint_names() const { return robot_->model().names; }
            const pinocchio::SE3& model_joint_pos(const std::string& joint_name) const
            {
                assert(tsid_);
                assert(robot_);
                return robot_->position(tsid_->data(), robot_->model().getJointId(joint_name));
            }
            // parameters that can be optimized / tuned online
            // (e.g., weights of task, gains of the tasks, etc.)
            virtual const opt_params_t& opt_params() const
            {
                IWBC_ERROR("calling opt_params but no param to set in base controller");
                static opt_params_t x;
                return x;
            }
            double cost(const std::shared_ptr<tsid::tasks::TaskBase>& task)
            {
                assert(task);
                return (task->getConstraint().matrix() * ddq_ - task->getConstraint().vector()).norm();
            }
            virtual std::shared_ptr<tsid::tasks::TaskBase> task(const std::string& task_name) = 0;
            double cost(const std::string& task_name) { return cost(task(task_name)); }

        private:
            std::vector<int> get_non_mimics_indexes() const;

        protected:
            void _reset();
            void _solve();

            std::shared_ptr<tsid::tasks::TaskComEquality> make_com_task(const std::string& name, double kp, const tsid::math::Vector& mask = cartesian_mask::xyz) const;
            std::shared_ptr<tsid::tasks::TaskJointPosture> make_posture_task(const std::string& name, double kp, const tsid::math::Vector& mask = {}) const;
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
        using Register = Factory::AutoRegister<T>;
    } // namespace controllers
} // namespace inria_wbc
#endif