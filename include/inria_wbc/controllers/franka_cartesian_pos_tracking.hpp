#ifndef IWBC_POS_TRACKING_HPP
#define IWBC_POS_TRACKING_HPP

#include <inria_wbc/controllers/controller.hpp>
#include <inria_wbc/estimators/cop.hpp>

namespace inria_wbc {
    namespace controllers {
        namespace cst {

            static constexpr char endEffector_joint_name[] = "panda_hand_joint"; //ee joint

        }; // namespace cst

        class FrankaCartPosTracking: public Controller {
        public:
            FrankaCartPosTracking(const Params& params);
            FrankaCartPosTracking(const FrankaCartPosTracking& other) = delete;
            FrankaCartPosTracking& operator=(const FrankaCartPosTracking& o) const = delete;
            virtual ~FrankaCartPosTracking(){};

            virtual void update(const SensorData& sensor_data) override;

            virtual std::shared_ptr<tsid::tasks::TaskBase> task(const std::string& task_name) override;

            void set_posture_ref(const tsid::math::Vector& ref);

            pinocchio::SE3 get_se3_ref(const std::string& task_name);

            void set_se3_ref(const pinocchio::SE3& ref, const std::string& task_name);

            void remove_task(const std::string& task_name, double transition_duration = 0.0);

            virtual const opt_params_t& opt_params() const override { return params_.opt_params; }

        protected:
            virtual void parse_configuration_yaml(const std::string& sot_config_path);
            virtual void set_stack_configuration();
            virtual void set_default_opt_params(std::map<std::string, double>& p);

            std::map<std::string, double> opt_params_; // the parameters that we can tune with an optimizer (e.g., task weights)
            std::string ref_config_ = "start";

            std::shared_ptr<tsid::tasks::TaskJointPosture> posture_task_;
            std::shared_ptr<tsid::tasks::TaskSE3Equality> cartesian_ee_;//~~ check if needed (maybe not) already stored in se3_tasks_
            // SE tasks (trajectories of joints/frames)
            std::unordered_map<std::string, std::shared_ptr<tsid::tasks::TaskSE3Equality>> se3_tasks_;

        };

    } // namespace controllers
} // namespace inria_wbc
#endif
