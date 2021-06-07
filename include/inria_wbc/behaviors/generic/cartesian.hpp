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
        class TalosMoveArm : public Behavior {
        public:
            TalosMoveArm(const controller_ptr_t& controller, const YAML::Node& config);
            TalosMoveArm() = delete;
            TalosMoveArm(const TalosMoveArm&) = delete;

            void update(const controllers::SensorData& sensor_data = {}) override;
            virtual ~TalosMoveArm() {}
            std::string behavior_type() const override { return controllers::behavior_types::DOUBLE_SUPPORT; };

        private:
            int time_ = 0;
            int traj_selector_ = 0;
            std::vector<std::vector<pinocchio::SE3>> trajectories_;
            std::vector<pinocchio::SE3> current_trajectory_;
            float trajectory_duration_ = 3; //will be changed if specified in yaml
            float motion_size_ = 0.05; //will be changed if specified in yaml
        };
    } // namespace behaviors
} // namespace inria_wbc
#endif