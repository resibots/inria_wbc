#ifndef IWBC_POS_TRACKER_HPP
#define IWBC_POS_TRACKER_HPP

#include <inria_wbc/controllers/controller.hpp>
#include <inria_wbc/estimators/cop.hpp>

namespace inria_wbc {
    namespace controllers {
      
        class FrankaPosTracker : public Controller {
        public:
            FrankaPosTracker(const Params& params);
            FrankaPosTracker(const FrankaPosTracker& other) = delete;
            FrankaPosTracker& operator=(const FrankaPosTracker& o) const = delete;
            virtual ~FrankaPosTracker(){};

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


            void set_task_ref(const tsid::math::Vector& ref, const std::string& task_name);
            void set_task_ref(const pinocchio::SE3& ref, const std::string& task_name);
            void get_task_ref(const std::string& task_name, tsid::math::Vector& vec);
            void get_task_ref(const std::string& task_name, pinocchio::SE3& se3);
     

            double cost(const std::string& task_name) const override { return Controller::cost(task<tsid::tasks::TaskBase>(task_name)); }
            std::shared_ptr<tsid::tasks::TaskSE3Equality> se3_task(const std::string& str) { return task<tsid::tasks::TaskSE3Equality>(str); }

            // this only removes the task from the TSID list of tasks (the task is not destroyed)
            // therefore you can re-add it later by using its name
            void remove_task(const std::string& task_name, double transition_duration = 0.0);

            // for external optimizers: we exclude the bound task for safety
            size_t num_task_weights() const;
            void update_task_weights(const std::vector<double>& new_weights);


            virtual void update(const SensorData& sensor_data = {}) override;

        protected:
            std::string m_ref_config;

            virtual void parse_tasks(const std::string&);

            // the list of all the tasks
            std::unordered_map<std::string, std::shared_ptr<tsid::tasks::TaskBase>> tasks_;
        
            std::map<std::string, double> opt_params_; // the parameters that we can tune with an optimizer (e.g., task weights)

        };

    } // namespace controllers
} // namespace inria_wbc
#endif
