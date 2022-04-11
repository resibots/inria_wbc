#include "inria_wbc/behaviors/humanoid/squat.hpp"

namespace inria_wbc {
    namespace behaviors {
        namespace humanoid {
            static Register<Squat> _squat("humanoid::squat");

            Squat::Squat(const controller_ptr_t& controller, const YAML::Node& config) : Behavior(controller, config)
            {
                auto pos_controller = std::dynamic_pointer_cast<controllers::PosTracker>(controller_);

                if (!pos_controller)
                    IWBC_ERROR("Squat behavior requires at least a PosTracker controller.");
                if (pos_controller->com_task()->getMask()(2) == 0)
                    IWBC_ERROR("Squat behavior requires mask of com task to be 1 on z-axis.");

                //////////////////// DEFINE COM TRAJECTORIES  //////////////////////////////////////
                traj_selector_ = 0;
                auto com_init = pos_controller->get_com_ref();

                auto c = IWBC_CHECK(config["BEHAVIOR"]);
                trajectory_duration_ = IWBC_CHECK(c["trajectory_duration"].as<float>());
                motion_size_ = IWBC_CHECK(c["motion_size"].as<float>());

                behavior_type_ = this->behavior_type();
                pos_controller->set_behavior_type(behavior_type_);

                Eigen::Vector3d com_final = com_init;
                com_final(2) -= motion_size_;

                trajectories_.push_back(
                    trajs::to_sample_trajectory(
                        trajs::min_jerk_trajectory(com_init, com_final, pos_controller->dt(), trajectory_duration_),
                        trajs::min_jerk_trajectory<trajs::d_order::FIRST>(com_init, com_final, pos_controller->dt(), trajectory_duration_),
                        trajs::min_jerk_trajectory<trajs::d_order::SECOND>(com_init, com_final, pos_controller->dt(), trajectory_duration_)));
                trajectories_.push_back(
                    trajs::to_sample_trajectory(
                        trajs::min_jerk_trajectory(com_final, com_init, pos_controller->dt(), trajectory_duration_),
                        trajs::min_jerk_trajectory<trajs::d_order::FIRST>(com_final, com_init, pos_controller->dt(), trajectory_duration_),
                        trajs::min_jerk_trajectory<trajs::d_order::SECOND>(com_final, com_init, pos_controller->dt(), trajectory_duration_)));
                current_trajectory_ = trajectories_[traj_selector_];
            }

            void Squat::update(const controllers::SensorData& sensor_data)
            {
                auto ref = current_trajectory_[time_];
                std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_com_ref(ref);

                controller_->update(sensor_data);
                time_++;
                if (time_ == current_trajectory_.size()) {
                    time_ = 0;
                    traj_selector_ = ++traj_selector_ % trajectories_.size();
                    current_trajectory_ = trajectories_[traj_selector_];
                }
            }
        } // namespace humanoid
    } // namespace behaviors
} // namespace inria_wbc
