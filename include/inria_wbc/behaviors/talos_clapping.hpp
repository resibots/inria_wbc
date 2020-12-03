#ifndef IWBC_MOVE_ARM_HPP
#define IWBC_MOVE_ARM_HPP
#include <chrono>
#include <iostream>
#include <signal.h>

#include <inria_wbc/behaviors/behavior.hpp>
#include <inria_wbc/controllers/pos_tracker.hpp>
#include <inria_wbc/utils/trajectory_handler.hpp>

namespace inria_wbc {
    namespace behaviors {
        class TalosClapping : public Behavior {
        public:
            TalosClapping(const controller_ptr_t& controller);
            TalosClapping() = delete;
            TalosClapping(const TalosClapping&) = delete;

            void update(const controllers::SensorData& sensor_data) override;
            virtual ~TalosClapping() {}

        private:
            std::vector<std::vector<pinocchio::SE3>> lh_trajs_;
            std::vector<std::vector<pinocchio::SE3>> rh_trajs_;
            int current_traj_ = 0;
            int time_ = 0;

            float trajectory_duration_ = 3; //will be changed if specified in yaml
            float motion_size_ = 0.05; //will be changed if specified in yaml
        };
    } // namespace behaviors
} // namespace inria_wbc
#endif