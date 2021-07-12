#ifndef IWBC_BEHAVIOR_GENERIC_CARTESIAN_HPP
#define IWBC_BEHAVIOR_GENERIC_CARTESIAN_HPP
#include <chrono>
#include <iostream>
#include <signal.h>

#include <inria_wbc/behaviors/behavior.hpp>
#include <inria_wbc/controllers/pos_tracker.hpp>
#include <inria_wbc/utils/trajectory_handler.hpp>

namespace inria_wbc {
    namespace behaviors {
        namespace generic {
            class Cartesian : public Behavior {
            public:
                Cartesian(const controller_ptr_t& controller, const YAML::Node& config);
                Cartesian() = delete;
                Cartesian(const Cartesian&) = delete;
                virtual ~Cartesian() {}

                void update(const controllers::SensorData& sensor_data = {}) override;
                std::string behavior_type() const override { return controllers::behavior_types::DOUBLE_SUPPORT; };

            private:
                int time_ = 0;
                int traj_selector_ = 0;
                std::vector<std::vector<std::vector<pinocchio::SE3>>> trajectories_;
                float trajectory_duration_; //will be changed if specified in yaml
                std::vector<std::string> task_names_; // from YAML
                std::vector<Eigen::Vector3d> relative_targets_; // form YAML
                bool loop_;
            };
        } // namespace generic
    } // namespace behaviors
} // namespace inria_wbc
#endif