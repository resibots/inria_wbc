#ifndef IWBC_HUMANOID_ACTIVEWALK_ON_SPOT
#define IWBC_HUMANOID_ACTIVEWALK_ON_SPOT
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
            class ActiveWalk : public Behavior {
            public:
                ActiveWalk(const controller_ptr_t& controller, const YAML::Node& config);
                ActiveWalk() = delete;
                ActiveWalk(const ActiveWalk& other) = default;
                void update(const controllers::SensorData& sensor_data = {}) override;

                std::string behavior_type() const override { return controllers::behavior_types::SINGLE_SUPPORT; };

                template <typename T>
                int sign(T val)
                {
                    return (T(0) < val) - (val < T(0));
                }

            private:
                int time_ = 0;
                float dt_;

                float traj_com_duration_ = 3; //will be changed if specified in yaml
                float traj_foot_duration_ = 3; //will be changed if specified in yaml
                float step_height_ = 0.1;
                float step_length_ = 0.1;

                Eigen::VectorXd com_init_, com_final_;
                pinocchio::SE3 lh_init_, lh_final_;
                pinocchio::SE3 rh_init_, rh_final_;
                pinocchio::SE3 lf_init_, lf_final_;
                pinocchio::SE3 rf_init_, rf_final_;

                int index_ = 0;
                bool begin_ = true;
                int state_ = -1;
                int next_state_ = States::GO_TO_MIDDLE;
                std::string left_ankle_name_;
                std::string right_ankle_name_;
                bool first_step_ = true;
                bool remove_contacts_ = true;
                int num_cycles_ = -1;
                int cycle_count_ = 0;
                bool go_to_middle_ = true;
                float com_percentage_ = 0.5;
                float left_sole_lyp_ = 0.0;
                float left_sole_lyn_ = 0.0;
                float right_sole_lyp_ = 0.0;
                float right_sole_lyn_ = 0.0;

                float force_treshold_ = inria_wbc::estimators::FMIN;

                enum States {
                    GO_TO_RF = 0,
                    LIFT_DOWN_LF = 1,
                    LIFT_UP_LF = 2,
                    GO_TO_LF = 3,
                    LIFT_DOWN_RF = 4,
                    LIFT_UP_RF = 5,
                    GO_TO_MIDDLE = 6
                };
            };
        } // namespace humanoid
    } // namespace behaviors
} // namespace inria_wbc
#endif