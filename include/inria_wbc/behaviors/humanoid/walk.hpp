#ifndef IWBC_HUMANOID_WALK_ON_SPOT
#define IWBC_HUMANOID_WALK_ON_SPOT
#include <chrono>
#include <iostream>
#include <signal.h>

#include <inria_wbc/behaviors/behavior.hpp>
#include <inria_wbc/controllers/talos_pos_tracker.hpp>
#include <inria_wbc/estimators/cop.hpp>
#include <inria_wbc/trajs/trajectory_generator.hpp>

namespace inria_wbc {
    namespace behaviors {
        namespace humanoid {
            class Walk : public Behavior {
            public:
                Walk(const controller_ptr_t& controller, const YAML::Node& config);
                Walk() = delete;
                Walk(const Walk& other) = default;
                void update(const controllers::SensorData& sensor_data = {}) override;

                std::string behavior_type() const override { return controllers::behavior_types::SINGLE_SUPPORT; };

            private:
                void _generate_trajectories(int num_of_cycles);
                int time_ = 0;
                float dt_;
                int _current_traj = 0;
                Eigen::VectorXd _last_com;
                pinocchio::SE3 _last_lf;
                pinocchio::SE3 _last_rf;

                std::vector<std::vector<Eigen::VectorXd>> _com_trajs;
                std::vector<std::vector<pinocchio::SE3>> _lf_trajs;
                std::vector<std::vector<pinocchio::SE3>> _rf_trajs;

                std::vector<std::vector<pinocchio::SE3>> _lh_trajs;
                std::vector<std::vector<pinocchio::SE3>> _rh_trajs;


                float traj_com_duration_ = 3; //will be changed if specified in yaml
                float traj_foot_duration_ = 3; //will be changed if specified in yaml
                float step_height_ = 0.1;
                float step_length_ = 0.1;
                int num_of_cycles_ = 10;
                bool run_ = true;
                // State machine stats for walking on the spot cycle
                int state_ = -1;
                enum States {
                    INIT = 0,
                    LF_INIT = 1,
                    LIFT_DOWN_LF = 2,
                    MOVE_COM_LEFT = 3,
                    LIFT_UP_RF = 4,
                    LIFT_DOWN_RF = 5,
                    MOVE_COM_RIGHT = 6,
                    LIFT_UP_LF = 7,
                    LIFT_DOWN_LF_FINAL = 8,
                    MOVE_COM_CENTER_FINAL = 9
                };
                std::vector<States> cycle_;
            };
        } // namespace humanoid
    } // namespace behaviors
} // namespace inria_wbc
#endif