#ifndef TALOS_BEHAVIOR_HPP
#define TALOS_BEHAVIOR_HPP

#include "controllers/talos_base_controller.hpp"

namespace tsid_sot
{
    namespace behaviors
    {
        class Behavior
        {
        public:
            Behavior(const std::shared_ptr<tsid_sot::controllers::TalosBaseController>& controller) :
                controller_(controller){}

            virtual Eigen::VectorXd cmd() = 0;

            virtual std::shared_ptr<tsid_sot::controllers::TalosBaseController> controller() { return controller_; };

        protected:
            std::shared_ptr<tsid_sot::controllers::TalosBaseController> controller_;
        };
    }
}
#endif

