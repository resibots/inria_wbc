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

            virtual Eigen::VectorXd cmd() = 0;

            virtual std::shared_ptr<inria_wbc::controllers::TalosBaseController> controller() { return controller_; };

        protected:
            std::shared_ptr<inria_wbc::controllers::TalosBaseController> controller_;
        };
    }
}
#endif

