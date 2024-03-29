#ifndef IWBC_BEHAVIOR_GENERIC_MoveFeet_HPP
#define IWBC_BEHAVIOR_GENERIC_MoveFeet_HPP
#include <chrono>
#include <iostream>
#include <signal.h>

#include <inria_wbc/behaviors/behavior.hpp>
#include <inria_wbc/controllers/pos_tracker.hpp>
#include <inria_wbc/trajs/trajectory_generator.hpp>

namespace inria_wbc {
    namespace behaviors {
        namespace generic {
            class MoveFeet : public Behavior {
            public:
                MoveFeet(const controller_ptr_t& controller, const YAML::Node& config);
                MoveFeet() = delete;
                MoveFeet(const MoveFeet&) = delete;
                virtual ~MoveFeet() {}

                void update(const controllers::SensorData& sensor_data = {}) override;
                std::string behavior_type() const override { return controllers::behavior_types::DOUBLE_SUPPORT; };

            private:
                int time_ = 0;
                int traj_selector_ = 0;
                std::vector<std::vector<std::vector<pinocchio::SE3>>> trajectories_;
                std::vector<std::vector<std::vector<Eigen::VectorXd>>> trajectories_d_;
                std::vector<std::vector<std::vector<Eigen::VectorXd>>> trajectories_dd_;

                float trajectory_duration_; // from YAML
                std::vector<std::string> task_names_; // from YAML
                std::vector<std::string> contact_names_; // from YAML
                bool loop_; // from YAML
            };
        } // namespace generic
    } // namespace behaviors
} // namespace inria_wbc
#endif