#ifndef IWBC_BEHAVIOR_HPP
#define IWBC_BEHAVIOR_HPP

#include <inria_wbc/controllers/talos_base_controller.hpp>

namespace inria_wbc
{
    namespace behaviors
    {
        class Behavior
        {
        public:
            Behavior(const std::shared_ptr<inria_wbc::controllers::TalosBaseController>& controller) :
                controller_(controller){}
            
            Behavior(const Behavior& other) { controller_ = other.controller()->clone(); }
            virtual ~Behavior(){}
            virtual std::shared_ptr<Behavior> clone() = 0;
            virtual bool cmd(Eigen::VectorXd &) = 0;

            virtual std::shared_ptr<inria_wbc::controllers::TalosBaseController> controller() { return controller_; };
            virtual std::shared_ptr<const inria_wbc::controllers::TalosBaseController> controller() const { return controller_; };

        protected:
            std::shared_ptr<inria_wbc::controllers::TalosBaseController> controller_;
        };
    }
}
#endif

