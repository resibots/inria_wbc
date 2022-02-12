#ifndef IWBC_HUMANOID_SQUAT_HPP
#define IWBC_HUMANOID_SQUAT_HPP
#include <chrono>
#include <iostream>
#include <signal.h>

#include <inria_wbc/behaviors/behavior.hpp>
#include <inria_wbc/controllers/pos_tracker.hpp>
#include <inria_wbc/trajs/trajectory_generator.hpp>

namespace inria_wbc {
    namespace behaviors {
        namespace humanoid {
            class Squat : public Behavior {
            public:
                Squat(const controller_ptr_t& controller, const YAML::Node& node);
                Squat() = delete;
                Squat(const Squat& other) = delete;
                virtual ~Squat() {}

                void update(const controllers::SensorData& sensor_data = {}) override;
                std::string behavior_type() const override { return controllers::behavior_types::DOUBLE_SUPPORT; };

            private:
                int time_ = 0;
                int traj_selector_ = 0;
                std::vector<std::vector<tsid::trajectories::TrajectorySample>> trajectories_;
                std::vector<tsid::trajectories::TrajectorySample> current_trajectory_;
                float trajectory_duration_ = 3; //will be changed if specified in yaml
                float motion_size_ = 0.2; //will be changed if specified in yaml
            };
        } // namespace humanoid
    } // namespace behaviors
} // namespace inria_wbc
#endif