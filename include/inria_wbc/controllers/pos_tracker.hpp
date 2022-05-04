#ifndef IWBC_POS_TRACKER_HPP
#define IWBC_POS_TRACKER_HPP

#include <inria_wbc/controllers/controller.hpp>
#include <inria_wbc/estimators/cop.hpp>
#include <inria_wbc/trajs/utils.hpp>

namespace inria_wbc {
    namespace controllers {
      
        // generic position tracker, for both fixed-base and floatin-base robots
        class PosTracker : public Controller {
        public:
            PosTracker(const YAML::Node& config);
            PosTracker(const PosTracker& other) = delete;
            PosTracker& operator=(const PosTracker& o) const = delete;
            virtual ~PosTracker(){};

            const std::unordered_map<std::string, std::shared_ptr<tsid::tasks::TaskBase>>& tasks() const { return tasks_; }
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
            std::shared_ptr<tsid::contacts::Contact6dExt> contact(const std::string& str) const
            {
                auto it = contacts_.find(str);
                IWBC_ASSERT(it != contacts_.end(), "Contact [", str, "] not found");
                return it->second;
            }
            bool has_task(const std::string& str) const { return tasks_.find(str) != tasks_.end(); }
            bool has_contact(const std::string& str) const { return contacts_.find(str) != contacts_.end(); }
            std::shared_ptr<tsid::tasks::TaskJointPosVelAccBounds> bound_task() { return task<tsid::tasks::TaskJointPosVelAccBounds>("bounds"); }
            std::shared_ptr<tsid::tasks::TaskMEquality> momentum_task() { return task<tsid::tasks::TaskMEquality>("momentum"); }
            std::shared_ptr<tsid::tasks::TaskComEquality> com_task() { return task<tsid::tasks::TaskComEquality>("com"); }
            std::shared_ptr<tsid::tasks::TaskSE3Equality> se3_task(const std::string& str) { return task<tsid::tasks::TaskSE3Equality>(str); }

            double cost(const std::string& task_name) const override { return Controller::cost(task<tsid::tasks::TaskBase>(task_name)); }
            double objective_value() const { return solver_->getObjectiveValue(); }

            pinocchio::SE3 get_se3_ref(const std::string& task_name);
            tsid::trajectories::TrajectorySample get_full_se3_ref(const std::string& task_name) { return se3_task(task_name)->getReference(); }
            // we do not return the velocity for now
            const tsid::math::Vector3 get_com_ref() { return com_task()->getReference().getValue(); }
            const tsid::trajectories::TrajectorySample get_full_com_ref() { return com_task()->getReference(); }
            const tsid::trajectories::TrajectorySample get_full_momentum_ref() { return momentum_task()->getReference(); }
            void set_com_ref(const tsid::math::Vector3& ref) { com_task()->setReference(trajs::to_sample(ref)); }
            void set_com_ref(const tsid::trajectories::TrajectorySample& sample) { com_task()->setReference(sample); }
            void set_momentum_ref(const tsid::trajectories::TrajectorySample& sample) { momentum_task()->setReference(sample); }
            void set_se3_ref(const pinocchio::SE3& ref, const std::string& task_name);
            void set_contact_se3_ref(const pinocchio::SE3& ref, const std::string& contact_name);
            void set_se3_ref(tsid::trajectories::TrajectorySample& sample, const std::string& task_name);

            void remove_contact(const std::string& contact_name);
            void add_contact(const std::string& contact_name, const int& motion_priority_level = 0);
            Eigen::VectorXd force_torque_from_solution(const std::string& foot);

            // this only removes the task from the TSID list of tasks (the task is not destroyed)
            // therefore you can re-add it later by using its name
            void remove_task(const std::string& task_name, double transition_duration = 0.0);
            void set_task_weight(const std::string& task_name, double w) { tsid_->updateTaskWeight(task_name, w); }
            // for external optimizers: we exclude the bound task for safety
            size_t num_task_weights() const;
            void update_task_weights(const std::vector<double>& new_weights);

        protected:
            virtual void parse_tasks(const std::string& path, const YAML::Node& config);
            virtual void parse_frames(const std::string&);
            // the list of all the tasks
            std::unordered_map<std::string, std::shared_ptr<tsid::tasks::TaskBase>> tasks_;
            std::vector<std::string> activated_tasks_;
            // contacts are not tasks in tsid
            std::unordered_map<std::string, std::shared_ptr<tsid::contacts::Contact6dExt>> contacts_;

            std::map<std::string, double> opt_params_; // the parameters that we can tune with an optimizer (e.g., task weights)
        };

    } // namespace controllers
} // namespace inria_wbc
#endif
