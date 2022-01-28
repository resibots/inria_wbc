#include "inria_wbc/behaviors/behavior.hpp"
#include "inria_wbc/controllers/pos_tracker.hpp"
#include "inria_wbc/controllers/tasks.hpp"

namespace inria_wbc {
    namespace behaviors {
        void Behavior::_customize_tasks(const controller_ptr_t& controller, const YAML::Node& config)
        {
            static const std::string yellow = "\x1B[33m";
            static const std::string rst = "\x1B[0m";
            auto node = config["BEHAVIOR"];
            if (node["customize_task_weights"]) {
                auto pos_tracker = std::dynamic_pointer_cast<controllers::PosTracker>(controller);
                IWBC_ASSERT(pos_tracker, "Task customization requires a controllers::PosTracker or a derivative");
                auto tasks = node["customize_task_weights"];
                for (auto it = tasks.begin(); it != tasks.end(); ++it) {
                    auto name = IWBC_CHECK(it->first.as<std::string>());
                    std::cout << yellow << "Warning:"  << " overriding weight of task: " << name << rst << std::endl;
                    pos_tracker->set_task_weight(name, it->second.as<double>());
                }
            }
        }
    } // namespace behaviors
} // namespace inria_wbc