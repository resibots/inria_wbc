#ifndef IWBC_POS_TRACKER_HPP
#define IWBC_POS_TRACKER_HPP

#include <inria_wbc/controllers/controller.hpp>
#include <inria_wbc/estimators/cop.hpp>

namespace inria_wbc {
    namespace controllers {
      
        class PosTracker : public Controller {
        public:
            PosTracker(const Params& params);
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

            // typically, for external optimizers (or other use?)
            // the default weights are those of the yaml
            void set_task_weight(const std::string& task_name, double weight);
           
        protected:
            virtual void parse_tasks(const std::string&);
            virtual void parse_frames(const std::string&);
            // the list of all the tasks
            std::unordered_map<std::string, std::shared_ptr<tsid::tasks::TaskBase>> tasks_;
            // contacts are not tasks in tsid
            std::unordered_map<std::string, std::shared_ptr<tsid::contacts::Contact6d>> contacts_;

        
            std::map<std::string, double> opt_params_; // the parameters that we can tune with an optimizer (e.g., task weights)
       
        };

    } // namespace controllers
} // namespace inria_wbc
#endif
