#ifndef IWBC_POS_TRACKING_HPP
#define IWBC_POS_TRACKING_HPP

#include <tsid/contacts/contact-6d.hpp>
#include <tsid/contacts/contact-point.hpp>
#include <tsid/math/utils.hpp>
#include <tsid/tasks/task-actuation-bounds.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-joint-bounds.hpp>
#include <tsid/tasks/task-joint-posVelAcc-bounds.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/trajectories/trajectory-base.hpp>

#include <inria_wbc/controllers/controller.hpp>

namespace inria_wbc {
    namespace controllers {
        namespace cst {
            // contact configuration
            static constexpr double lxp = 0.1; // foot length in positive x direction
            static constexpr double lxn = 0.11; // foot length in negative x direction
            static constexpr double lyp = 0.069; // foot length in positive y direction
            static constexpr double lyn = 0.069; // foot length in negative y direction
            static constexpr double lz = 0.107; // foot sole height with respect to ankle joint
            static constexpr double mu = 0.3; // friction coefficient
            static constexpr double fMin = 5.0; // minimum normal force
            static constexpr double fMax = 1500.0; // maximum normal force
            static tsid::math::Vector3 contact_normal = tsid::math::Vector3::UnitZ();
            // frame & joint names
            static constexpr char rf_joint_name[] = "leg_right_6_joint"; // right foot joint name
            static constexpr char lf_joint_name[] = "leg_left_6_joint"; // left foot joint name
            static constexpr char rh_joint_name[] = "gripper_right_joint"; // left foot joint name

            static constexpr char lh_joint_name[] = "gripper_left_joint"; // left foot joint name

            static constexpr char torso_frame_name[] = "torso_2_link"; // left foot joint name
        }; // namespace cst

        class TalosPosTracking : public Controller {
        public:
            TalosPosTracking(const Params& params);
            TalosPosTracking(const TalosPosTracking& other) = delete;
            TalosPosTracking& operator=(const TalosPosTracking& o) const = delete;
            virtual ~TalosPosTracking(){};

            std::shared_ptr<tsid::tasks::TaskComEquality> com_task() { return com_task_; }
            pinocchio::SE3 get_se3_ref(const std::string& task_name);

            void set_com_ref(const tsid::math::Vector3& ref);
            void set_posture_ref(const tsid::math::Vector& ref);
            void set_se3_ref(const pinocchio::SE3& ref, const std::string& task_name);

            void remove_contact(const std::string& contact_name);
            void add_contact(const std::string& contact_name);

            virtual const opt_params_t& opt_params() const override { return params_.opt_params; }

        protected:
            virtual void parse_configuration_yaml(const std::string& sot_config_path);
            virtual void set_stack_configuration();
            virtual void set_default_opt_params(std::map<std::string, double>& p);

            std::shared_ptr<tsid::contacts::Contact6d> make_contact_task(const std::string& name, const std::string frame_name, double kp) const;
            std::shared_ptr<tsid::tasks::TaskComEquality> make_com_task(const std::string& name, double kp) const;
            std::shared_ptr<tsid::tasks::TaskJointPosture> make_posture_task(const std::string& name, double kp) const;
            std::shared_ptr<tsid::tasks::TaskSE3Equality> make_torso_task(const std::string& name, const std::string& frame_name, double kp) const;
            std::shared_ptr<tsid::tasks::TaskSE3Equality> make_floatingb_task(const std::string& name, const std::string& joint_name, double kp) const;
            std::shared_ptr<tsid::tasks::TaskSE3Equality> make_hand_task(const std::string& name, const std::string& joint_name, double kp) const;
            std::shared_ptr<tsid::tasks::TaskSE3Equality> make_foot_task(const std::string& name, const std::string& joint_name, double kp) const;
            std::shared_ptr<tsid::tasks::TaskJointPosVelAccBounds> make_bound_task(const std::string& name) const;

            std::map<std::string, double> opt_params_; // the parameters that we can tune with an optimizer (e.g., task weights)

            // contacts
            std::shared_ptr<tsid::contacts::Contact6d> contactRF_;
            std::shared_ptr<tsid::contacts::Contact6d> contactLF_;

            // posture
            std::shared_ptr<tsid::tasks::TaskJointPosture> posture_task_;

            // SE tasks (trajectories of joints/frames)
            std::shared_ptr<tsid::tasks::TaskComEquality> com_task_;
            std::shared_ptr<tsid::tasks::TaskSE3Equality> floatingb_task_;
            std::unordered_map<std::string, std::shared_ptr<tsid::tasks::TaskSE3Equality>> se3_tasks_;

            // bounds
            std::shared_ptr<tsid::tasks::TaskJointPosVelAccBounds> bounds_task_;
        };

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

    } // namespace controllers
} // namespace inria_wbc
#endif
