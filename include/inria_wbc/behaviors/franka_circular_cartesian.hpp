#ifndef IWBC_SWITCH_FOOT
#define IWBC_SWITCH_FOOT
#include <chrono>
#include <iostream>
#include <signal.h>

#include <inria_wbc/behaviors/behavior.hpp>
#include <inria_wbc/controllers/franka_pos_tracker.hpp>
#include <inria_wbc/estimators/cop.hpp>
#include <inria_wbc/utils/trajectory_handler.hpp>

namespace inria_wbc {
    namespace behaviors {

        class CircCartTraj : public Behavior {
        public:
            CircCartTraj(const controller_ptr_t& controller);
            CircCartTraj() = delete;
            CircCartTraj(const CircCartTraj& other) = default;
            void update(const controllers::SensorData& sensor_data) override;

        private:
            float dt_;
 
            void _generate_trajectories();
        };
    } // namespace behaviors
} // namespace inria_wbc
#endif
