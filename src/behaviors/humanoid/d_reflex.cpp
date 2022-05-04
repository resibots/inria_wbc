#include "inria_wbc/behaviors/humanoid/d_reflex.hpp"

namespace inria_wbc {
    namespace behaviors {
        namespace humanoid {
            static Register<D_Reflex> __talos_d_reflex("humanoid::d_reflex");

            D_Reflex::D_Reflex(const controller_ptr_t& controller, const YAML::Node& config) : Behavior(controller, config)
            {

                auto tracker = std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller_);
                IWBC_ASSERT(tracker, "we need a pos tracker here");
                YAML::Node c = IWBC_CHECK(config["BEHAVIOR"]);

                activate_reflex_time_ = IWBC_CHECK(c["activate_reflex_time"].as<bool>());
                reflex_time_ = IWBC_CHECK(c["reflex_time"].as<float>());

                behavior_type_ = this->behavior_type();
                controller_->set_behavior_type(behavior_type_);
            }

            void D_Reflex::update(const controllers::SensorData& sensor_data)
            {
                auto controller = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_);
                controller->update(sensor_data);
                if ((activate_reflex_time_ && (reflex_time_ <= controller->t() && controller->t() < (reflex_time_ + controller->dt())) || activate_) && !already_activated_) {
                    std::cout << "Trigger D-Reflex at " << controller->t() << std::endl;

                    auto zero = Eigen::VectorXd::Zero(6);

                    auto rh_task = controller->se3_task("rh");
                    rh_task->Kp(zero);
                    rh_task->Kd(zero);

                    auto lh_task = controller->se3_task("lh");
                    // lh_task->Kp(zero);
                    // lh_task->Kd(zero);

                    controller->add_contact("contact_rhand");
                    already_activated_ = true;
                }
                time_++;
            }
        } // namespace humanoid
    } // namespace behaviors
} // namespace inria_wbc