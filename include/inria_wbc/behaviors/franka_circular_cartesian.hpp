#ifndef IWBC_SWITCH_FOOT
#define IWBC_SWITCH_FOOT
#include <chrono>
#include <iostream>
#include <signal.h>

#include <inria_wbc/behaviors/behavior.hpp>
#include <inria_wbc/controllers/pos_tracker.hpp>
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
            pinocchio::SE3 func_traj( const float t);

        private:
            int traj_index_;
            float dt_;
            float pitch_angle_;
            float radius_;
            float traj_cycle_duration_;
            Eigen::Vector3d xyz_offset_;
            std::vector<pinocchio::SE3> trajectory_;
            int num_traj_steps_;
        };
    } // namespace behaviors
} // namespace inria_wbc
#endif
