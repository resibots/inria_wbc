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

            void register_behavior(const std::string &behavior_name, behavior_creator_t pfn_create_behavior)
            {
                if (behavior_map_.find(behavior_name) == behavior_map_.end())
                {
                    behavior_map_[behavior_name] = std::make_pair(pfn_create_behavior, std::shared_ptr<Behavior>());
                }
                else
                {
                    std::cout << "Warning : there is already a " << behavior_name << " behavior in the factory" << std::endl;
                }
            }
            
            behavior_ptr_t create(const std::string &behavior_name, const controllers::TalosBaseController::Params& params = controllers::TalosBaseController::Params())
            {
                // std::optional is c++17 only, this is why we use boost for now
                auto it = behavior_map_.find(behavior_name);
                if (it != behavior_map_.end() && it->second.second)
                {
                    // WARNING: params are ignored for the 2nd object!
                    return it->second.second->clone();
                }
                else if(it != behavior_map_.end())
                {
                    auto behavior = it->second.first(params);
                    it->second.second = behavior;
                    return behavior;
                }
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
            Factory() {}
            Factory &operator=(const Factory &) { return *this; }
            std::map<std::string, std::pair<behavior_creator_t, std::shared_ptr<Behavior>>> behavior_map_;
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