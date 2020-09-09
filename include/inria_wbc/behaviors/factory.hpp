#ifndef IWBC_FACTORY_HPP
#define IWBC_FACTORY_HPP

#include <inria_wbc/behaviors/behavior.hpp>

namespace inria_wbc
{
    namespace behaviors
    {
        class Factory
        {
        public:
            ~Factory() { behavior_map_.clear(); }

            static Factory &instance()
            {
                static Factory instance;
                return instance;
            }
            typedef std::shared_ptr<Behavior> behavior_ptr_t;
            typedef std::function<behavior_ptr_t(const inria_wbc::controllers::TalosBaseController::Params &)>
                behavior_creator_t;
            typedef inria_wbc::controllers::TalosBaseController::Params behavior_params_t;

            void register_behavior(const std::string &behavior_name, behavior_creator_t pfn_create_behavior)
            {
                if (behavior_map_.find(behavior_name) == behavior_map_.end())
                {
                    BehaviorInfo info = {.creator_function = pfn_create_behavior};
                    behavior_map_[behavior_name] = info;
                }
                else
                {
                    std::cout << "Warning : there is already a " << behavior_name << " behavior in the factory" << std::endl;
                }
            }
            // standard creation : one new object each time
            behavior_ptr_t create(const std::string &behavior_name, const behavior_params_t& params)
            {
                auto it = behavior_map_.find(behavior_name);
                if(it != behavior_map_.end())
                {
                    auto behavior = it->second.creator_function(params);
                    it->second.params = params;
                    it->second.prototype = behavior->clone();
                    return behavior;
                }
                else
                    std::cerr << "Error :  " << behavior_name << " is not in the behavior factory" << std::endl;
                return behavior_ptr_t();
            }

            // when you create many identical objects:
            // first call, create the object like create()
            // second time, return a (clean, reset) clone to avoid parsing costs
            // WARNING: the params are ignored the second time!
            behavior_ptr_t create_or_clone(const std::string &behavior_name, const behavior_params_t& params=behavior_params_t())
            {
                // std::optional is c++17 only, this is why we use boost for now
                auto it = behavior_map_.find(behavior_name);
                if (it != behavior_map_.end() && it->second.prototype)
                    return it->second.prototype->clone();
                else if(it != behavior_map_.end())
                   return create(behavior_name, params);
                else
                    std::cerr << "Error :  " << behavior_name << " is not in the behavior factory" << std::endl;
                return behavior_ptr_t();
            }

            void controller_names()
            {
                for (auto &it : behavior_map_)
                {
                    std::cout << it.first << std::endl;
                }
            }

        private:
            struct BehaviorInfo {
                behavior_creator_t creator_function;
                behavior_ptr_t prototype;
                behavior_params_t params;
            };
            Factory() {}
            Factory &operator=(const Factory &) { return *this; }
            std::map<std::string, BehaviorInfo> behavior_map_;
        };
        
        template <typename BehaviorClass>
        struct AutoRegister
        {
            AutoRegister(std::string behavior_name)
            {
                Factory::instance().register_behavior(behavior_name, [](const inria_wbc::controllers::TalosBaseController::Params &params) {
                    return std::make_shared<BehaviorClass>(params);
                });
            }
        };

    } // namespace behavior
} // namespace inria_wbc
#endif