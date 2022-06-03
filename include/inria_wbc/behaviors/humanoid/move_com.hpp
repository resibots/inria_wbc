#ifndef IWBC_BEHAVIOR_HUMANOID_MOVE_COM_HPP
#define IWBC_BEHAVIOR_HUMANOID_MOVE_COM_HPP
#include <iostream>

#include <inria_wbc/behaviors/behavior.hpp>
#include <inria_wbc/controllers/pos_tracker.hpp>
#include <inria_wbc/trajs/trajectory_generator.hpp>

namespace inria_wbc {
    namespace behaviors {
        namespace humanoid {
            class MoveCom : public Behavior {
            public:
                MoveCom(const controller_ptr_t& controller, const YAML::Node& config);
                MoveCom() = delete;
                MoveCom(const MoveCom&) = delete;
                virtual ~MoveCom() {}

                void update(const controllers::SensorData& sensor_data = {}) override;
                std::string behavior_type() const override { return controllers::behavior_types::DOUBLE_SUPPORT; };

            private:
                bool loop_ = false;
                int time_ = 0;
                std::vector<Eigen::VectorXd> trajectory_;
                std::vector<Eigen::VectorXd> trajectory_d_;
                std::vector<Eigen::VectorXd> trajectory_dd_;
            };
        } // namespace humanoid
    } // namespace behaviors
} // namespace inria_wbc
#endif