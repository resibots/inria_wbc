#ifndef IWBC_BEHAVIOR_HPP
#define IWBC_BEHAVIOR_HPP

#include <inria_wbc/controllers/controller.hpp>
#include <inria_wbc/utils/factory.hpp>

namespace inria_wbc {
    namespace behaviors {
        class Behavior {
        public:
            using params_t = inria_wbc::controllers::Controller::params_t;
            Behavior(const std::shared_ptr<controllers::Controller>& controller) : controller_(controller) {}
            virtual ~Behavior() {}
            virtual bool update() = 0;
            virtual std::shared_ptr<controllers::Controller> controller() { return controller_; };
            virtual std::shared_ptr<const controllers::Controller> controller() const { return controller_; };

        protected:
            std::shared_ptr<inria_wbc::controllers::Controller> controller_;
        };
        using Factory = utils::Factory<Behavior>;
        template <typename T>
        struct Register : public utils::AutoRegister<Behavior, T> {
            Register(const std::string& name) : utils::AutoRegister<Behavior, T>(name) {}
        };

    } // namespace behaviors
} // namespace inria_wbc
#endif
