#ifndef IWBC_BEHAVIOR_HPP
#define IWBC_BEHAVIOR_HPP

#include <inria_wbc/controllers/controller.hpp>
#include <inria_wbc/utils/factory.hpp>

namespace inria_wbc {
    namespace behaviors {
        class Behavior {
        public:
            using controller_ptr_t = std::shared_ptr<inria_wbc::controllers::Controller>;
            Behavior(const controller_ptr_t& controller) : controller_(controller) { assert(controller); };
            virtual ~Behavior() {}
            virtual bool update() = 0;
            virtual std::shared_ptr<controllers::Controller> controller() { return controller_; };
            virtual std::shared_ptr<const controllers::Controller> controller() const { return controller_; };

        protected:
            std::shared_ptr<inria_wbc::controllers::Controller> controller_;
        };
        using Factory = utils::Factory<Behavior, Behavior::controller_ptr_t>;
        template <typename T>
        struct Register : public utils::AutoRegister<Behavior, T, Behavior::controller_ptr_t> {
            Register(const std::string& name) : utils::AutoRegister<Behavior, T, Behavior::controller_ptr_t>(name) {}
        };

    } // namespace behaviors
} // namespace inria_wbc
#endif
