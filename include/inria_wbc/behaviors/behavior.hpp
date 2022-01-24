#ifndef IWBC_BEHAVIOR_HPP
#define IWBC_BEHAVIOR_HPP

#include <inria_wbc/controllers/controller.hpp>
#include <inria_wbc/utils/factory.hpp>

namespace inria_wbc {
    namespace behaviors {
        class Behavior {
        public:
            using controller_ptr_t = std::shared_ptr<inria_wbc::controllers::Controller>;
            Behavior(const controller_ptr_t& controller, const YAML::Node& config) : 
                controller_(controller) { 
                    IWBC_ASSERT(controller, "Invalid controller pointer"); 
                    _customize_tasks(controller, config);
                };
            virtual ~Behavior() {}
            virtual void update(const controllers::SensorData& sensor_data = {}) = 0;
            virtual std::shared_ptr<controllers::Controller> controller() { return controller_; };
            virtual std::shared_ptr<const controllers::Controller> controller() const { return controller_; };
            virtual std::string behavior_type() const = 0;

        protected:
            void _customize_tasks(const controller_ptr_t& controller, const YAML::Node& config);
            std::shared_ptr<inria_wbc::controllers::Controller> controller_;
            std::string behavior_type_;
        };
        using Factory = utils::Factory<Behavior, Behavior::controller_ptr_t, const YAML::Node&>;
        template <typename T>
        using Register = Factory::AutoRegister<T>;
    } // namespace behaviors
} // namespace inria_wbc
#endif
