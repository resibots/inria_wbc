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

        class CircularCartesianTrajectory : public Behavior {
        public:
            CircularCartesianTrajectory(const controller_ptr_t& controller, const YAML::Node& config);
            CircularCartesianTrajectory() = delete;
            CircularCartesianTrajectory(const CircularCartesianTrajectory&) = delete;

            void update(const controllers::SensorData& sensor_data) override;
            virtual ~CircularCartesianTrajectory() {}

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
