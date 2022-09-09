#ifndef IWBC_HUMANOID_ACTIVEWALK2_HPP
#define IWBC_HUMANOID_ACTIVEWALK2_HPP
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
            class ActiveWalk2 : public Behavior {
            public:
                ActiveWalk2(const controller_ptr_t& controller, const YAML::Node& config);
                ActiveWalk2() = delete;
                ActiveWalk2(const ActiveWalk2& other) = default;
                void update(const controllers::SensorData& sensor_data = {}) override;

                std::string behavior_type() const override { return controllers::behavior_types::SINGLE_SUPPORT; };

                void set_se3_ref(const pinocchio::SE3& init, const pinocchio::SE3& final, const std::string& task_name, const std::string& contact_name, const double& trajectory_duration, const int& index);
                Eigen::VectorXd set_com_ref(const Eigen::VectorXd& init, const Eigen::VectorXd& final, const double& trajectory_duration, const int& index);

                template <typename T>
                int sign(T val)
                {
                    return (T(0) < val) - (val < T(0));
                }

            private:
                float dt_ = 0.0;

                float transition_duration_ = 3; //will be changed if specified in yaml
                float traj_foot_duration_ = 3; //will be changed if specified in yaml
                float step_height_ = 0.1;
                float step_length_ = 0.1;
                float step_lateral_ = 0.1;

                Eigen::VectorXd com_init_, com_final_;
                float com_foot_up_;
                pinocchio::SE3 lh_init_, lh_final_;
                pinocchio::SE3 rh_init_, rh_final_;
                pinocchio::SE3 lf_init_, lf_final_;
                pinocchio::SE3 rf_init_, rf_final_;
                tsid::math::Vector3 lh_delta_init_, rh_delta_init_;
                std::string torso_link_name_;

                int index_ = 0;
                int remove_contact_index_ = 0; //time index for when to remove the contact
                int time_index_ = 0;

                bool lift_foot_up_ = false;
                bool first_step_ = true;
                bool begin_ = true;
                int state_ = -1;
                int next_state_ = States::GO_TO_RF;

                std::string left_sole_name_;
                std::string right_sole_name_;
                bool remove_contacts_ = true;
                int num_cycles_ = -1;
                int cycle_count_ = 0;
                float com_percentage_ref_ = 0.5;
                float com_percentage_foot_up_ = 0.5;
                bool send_vel_acc_ = false;
                bool one_foot_ = false;
                float force_treshold_ = inria_wbc::estimators::FMIN;
                float error_cop_ = 0.0;
                float error_posture_ = 0.0;

                std::ofstream file_{"/home/pal/my_file"};
                int k = 0;

                bool activate_error_cop_ = false;

                enum States {
                    GO_TO_RF = 0,
                    MOVE_RF_FORWARD = 1,
                    GO_TO_MIDDLE_RF = 2,
                    LIFT_DOWN_LF = 3,
                    GO_TO_MIDDLE = 4,
                    GO_TO_LF = 5,
                    MOVE_LF_FORWARD = 6,
                    GO_TO_MIDDLE_LF = 7,
                    LIFT_DOWN_RF = 8
                };
            };
        } // namespace humanoid
    } // namespace behaviors
} // namespace inria_wbc
#endif
