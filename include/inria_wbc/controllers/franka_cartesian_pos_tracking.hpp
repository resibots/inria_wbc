#ifndef IWBC_POS_TRACKING_HPP
#define IWBC_POS_TRACKING_HPP

#include <inria_wbc/controllers/controller.hpp>
#include <inria_wbc/estimators/cop.hpp>

namespace inria_wbc {
    namespace controllers {
        namespace cst {

            static constexpr char endEffector_joint_name[] = "panda_hand_joint"; //ee joint

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

        class FrankaCartPosTracking: public Controller {
        public:
            FrankaCartPosTracking(const Params& params);
            FrankaCartPosTracking(const TalosPosTracking& other) = delete;
            FrankaCartPosTracking& operator=(const FrankaCartPosTracking& o) const = delete;
            virtual ~FrankaCartPosTracking(){};

            virtual void update(const SensorData& sensor_data) override;

            //~~???? HERE
            virtual const Eigen::Vector2d& cop() const override { return _cop_estimator.cop(); } //??
            virtual std::shared_ptr<tsid::tasks::TaskBase> task(const std::string& task_name) override;

            std::shared_ptr<tsid::tasks::TaskComEquality> com_task() { return com_task_; } // TODO remove (there is a generic accessor)
            pinocchio::SE3 get_se3_ref(const std::string& task_name);

            // TODO maybe a generic set_ref here
            void set_com_ref(const tsid::math::Vector3& ref);
            void set_posture_ref(const tsid::math::Vector& ref);
            void set_se3_ref(const pinocchio::SE3& ref, const std::string& task_name);

            void remove_contact(const std::string& contact_name);
            void add_contact(const std::string& contact_name);

            void remove_task(const std::string& task_name, double transition_duration = 0.0);

            virtual const opt_params_t& opt_params() const override { return params_.opt_params; }

        protected:
            virtual void parse_configuration_yaml(const std::string& sot_config_path);
            virtual void set_stack_configuration();
            virtual void set_default_opt_params(std::map<std::string, double>& p);
            std::shared_ptr<tsid::contacts::Contact6d> make_contact_task(const std::string& name, const std::string frame_name, double kp) const;

            estimators::Cop _cop_estimator;
            bool _use_stabilizer = true;
            Eigen::Vector2d _stabilizer_p = Eigen::Vector2d(0.005, 0.005);
            Eigen::Vector2d _stabilizer_d = Eigen::Vector2d(0, 0);

            std::map<std::string, double> opt_params_; // the parameters that we can tune with an optimizer (e.g., task weights)
            std::string ref_config_ = "pal_start";

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

    } // namespace controllers
} // namespace inria_wbc
#endif
