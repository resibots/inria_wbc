#ifndef IWBC_POS_TRACKING_HPP
#define IWBC_POS_TRACKING_HPP

#include <inria_wbc/controllers/controller.hpp>
#include <inria_wbc/estimators/cop.hpp>

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

            virtual void update(const SensorData& sensor_data) override;
            virtual const Eigen::Vector2d& cop() const override { return _cop_estimator.cop(); }

            template <typename T>
            std::shared_ptr<T> task(const std::string& str) const
            {
                auto it = tasks_.find(str);
                IWBC_ASSERT(it != tasks_.end(), "Task [", str, "] not found");
                auto t = std::dynamic_pointer_cast<T>(it->second);
                IWBC_ASSERT(t, "Task [", str, "] does not have the expected c++ type");
                IWBC_ASSERT(t->name() == it->first, "Task name error (tsid)[", t->name(), "] vs [", it->first, "]");
                return t;
            }
            std::shared_ptr<tsid::contacts::Contact6d> contact(const std::string &str) const {
                auto it = contacts_.find(str);
                IWBC_ASSERT(it != contacts_.end(), "Contact [", str, "] not found");
                return it->second;
            }
            std::shared_ptr<tsid::tasks::TaskComEquality> com_task() { return task<tsid::tasks::TaskComEquality>("com"); }
            std::shared_ptr<tsid::tasks::TaskSE3Equality> se3_task(const std::string& str) { return task<tsid::tasks::TaskSE3Equality>(str); }

            double cost(const std::string& task_name) const override { return Controller::cost(task<tsid::tasks::TaskBase>(task_name)); }

            pinocchio::SE3 get_se3_ref(const std::string& task_name);
            void set_com_ref(const tsid::math::Vector3& ref) { com_task()->setReference(to_sample(ref)); }
            void set_se3_ref(const pinocchio::SE3& ref, const std::string& task_name);

            void remove_contact(const std::string& contact_name);
            void add_contact(const std::string& contact_name);

            // this only removes the task from the TSID list of tasks (the task is not destroyed)
            // therefore you can re-add it later by using its name
            void remove_task(const std::string& task_name, double transition_duration = 0.0);

            virtual const opt_params_t& opt_params() const override { return params_.opt_params; }

        protected:
            virtual void parse_configuration_yaml(const std::string& sot_config_path);
            virtual void parse_tasks(const std::string& sot_config_path);

            virtual void set_stack_configuration();
            virtual void set_default_opt_params(std::map<std::string, double>& p);

            std::shared_ptr<tsid::contacts::Contact6d> make_contact_task(const std::string& name, const std::string frame_name, double kp) const;

            // the list of all the tasks
            std::unordered_map<std::string, std::shared_ptr<tsid::tasks::TaskBase>> tasks_;
            // contacts are not tasks in tsid
            std::unordered_map<std::string, std::shared_ptr<tsid::contacts::Contact6d>> contacts_;

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
