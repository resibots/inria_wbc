#include "inria_wbc/behaviors/talos_squat.hpp"

namespace inria_wbc {
    namespace behaviors {

        static Register<TalosSquat> __talos_squat("talos-squat");

        TalosSquat::TalosSquat(const controller_ptr_t& controller, const YAML::Node& config) : 
            Behavior(controller, config)
        {
            //////////////////// DEFINE COM TRAJECTORIES  //////////////////////////////////////
            traj_selector_ = 0;
            auto com_init = std::static_pointer_cast<controllers::PosTracker>(controller_)->get_com_ref();

            auto c = IWBC_CHECK(config["BEHAVIOR"]);
            trajectory_duration_ = IWBC_CHECK(c["trajectory_duration"].as<float>());
            motion_size_ = IWBC_CHECK(c["motion_size"].as<float>());

            auto com_final = com_init;
            com_final(2) -= motion_size_;

            trajectories_.push_back(trajectory_handler::compute_traj(com_init, com_final, controller_->dt(), trajectory_duration_));
            trajectories_.push_back(trajectory_handler::compute_traj(com_final, com_init, controller_->dt(), trajectory_duration_));
            current_trajectory_ = trajectories_[traj_selector_];
        }

        void TalosSquat::update(const controllers::SensorData& sensor_data)
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
    } // namespace behaviors
} // namespace inria_wbc
