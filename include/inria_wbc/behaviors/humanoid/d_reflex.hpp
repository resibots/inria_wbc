#ifndef IWBC_HUMANOID_D_REFLEX
#define IWBC_HUMANOID_D_REFLEX
#include <chrono>
#include <iostream>
#include <signal.h>

#include <inria_wbc/behaviors/behavior.hpp>
#include <inria_wbc/controllers/pos_tracker.hpp>
#include <inria_wbc/trajs/trajectory_generator.hpp>

namespace inria_wbc {
    namespace behaviors {
        namespace humanoid {
            class D_Reflex : public Behavior {
            public:
                D_Reflex(const controller_ptr_t& controller, const YAML::Node& config);
                D_Reflex() = delete;
                D_Reflex(const D_Reflex&) = delete;

                void update(const controllers::SensorData& sensor_data) override;
                virtual ~D_Reflex() {}
                std::string behavior_type() const override { return controllers::behavior_types::DOUBLE_SUPPORT; };

                void set_activate(bool activate) { activate_ = activate; };
                bool activate() { return activate_; };

            private:
                int time_ = 0;
                float reflex_time_ = 1000000.;
                bool activate_reflex_time_ = false;
                bool activate_ = false;
                bool already_activated_ = false;
            };
        } // namespace humanoid
    } // namespace behaviors
} // namespace inria_wbc
#endif