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
                std::vector<std::vector<pinocchio::SE3>> lf_trajectories_, rf_trajectories_;
                std::vector<std::vector<Eigen::VectorXd>> lf_trajectories_d_, rf_trajectories_d_;
                std::vector<std::vector<Eigen::VectorXd>> lf_trajectories_dd_, rf_trajectories_dd_;

                float trajectory_duration_; // from YAML
                std::string lf_task_name_, rf_task_name_;
                std::string lf_contact_name_, rf_contact_name_;
                int i_ = 0;
                bool loop_;
            };
        } // namespace generic
    } // namespace behaviors
} // namespace inria_wbc
#endif