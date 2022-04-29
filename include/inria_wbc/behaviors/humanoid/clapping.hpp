#ifndef IWBC_HUMANOID_CLAPPING
#define IWBC_HUMANOID_CLAPPING
#include <chrono>
#include <iostream>
#include <signal.h>

#include <inria_wbc/behaviors/behavior.hpp>
#include <inria_wbc/controllers/pos_tracker.hpp>
#include <inria_wbc/trajs/trajectory_generator.hpp>

namespace inria_wbc {
    namespace behaviors {
        namespace humanoid {
            class Clapping : public Behavior {
            public:
                Clapping(const controller_ptr_t& controller, const YAML::Node& config);
                Clapping() = delete;
                Clapping(const Clapping&) = delete;

                void update(const controllers::SensorData& sensor_data) override;
                virtual ~Clapping() {}
                std::string behavior_type() const override { return controllers::behavior_types::DOUBLE_SUPPORT; };

            private:
                std::vector<std::vector<pinocchio::SE3>> lh_trajs_;
                std::vector<std::vector<pinocchio::SE3>> rh_trajs_;
                int current_traj_ = 0;
                int time_ = 0;

                float trajectory_duration_ = 3; //will be changed if specified in yaml
                float motion_size_ = 0.05; //will be changed if specified in yaml
            };
        } // namespace humanoid
    } // namespace behaviors
} // namespace inria_wbc
#endif