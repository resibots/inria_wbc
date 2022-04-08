#ifndef IWBC_BEHAVIOR_GENERIC_CARTESIAN_TRAJ_HPP
#define IWBC_BEHAVIOR_GENERIC_CARTESIAN_TRAJ_HPP
#include <chrono>
#include <iostream>
#include <signal.h>

#include <inria_wbc/behaviors/behavior.hpp>
#include <inria_wbc/controllers/pos_tracker.hpp>
#include <inria_wbc/trajs/trajectory_generator.hpp>
#include <inria_wbc/trajs/loader.hpp>

namespace inria_wbc {
    namespace behaviors {
        namespace generic {
            class CartesianTraj : public Behavior {
            public:
                CartesianTraj(const controller_ptr_t& controller, const YAML::Node& config);
                CartesianTraj() = delete;
                CartesianTraj(const CartesianTraj&) = delete;
                virtual ~CartesianTraj() {}

                void update(const controllers::SensorData& sensor_data = {}) override;
                std::string behavior_type() const override { return controllers::behavior_types::DOUBLE_SUPPORT; };
            private:
                int time_ = 0;
                int step_ = 1;
                bool loop_;
                std::shared_ptr<trajs::Loader> traj_loader_;
                double scale_ = 1;
            };
        } // namespace generic
    } // namespace behaviors
} // namespace inria_wbc
#endif