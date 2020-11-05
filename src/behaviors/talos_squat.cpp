#include "inria_wbc/behaviors/talos_squat.hpp"

namespace inria_wbc {
    namespace behaviors {

        static Register<TalosSquat> __talos_squat("talos-squat");

        TalosSquat::TalosSquat(const controller_ptr_t& controller) : Behavior(controller)
        {
            //////////////////// DEFINE COM TRAJECTORIES  //////////////////////////////////////
            traj_selector_ = 0;
            auto com_init = std::static_pointer_cast<controllers::TalosPosTracking>(controller_)->get_pinocchio_com();

            YAML::Node config = YAML::LoadFile(controller_->params().sot_config_path);
            utils::parse(trajectory_duration_, "trajectory_duration", config, false, "BEHAVIOR");
            utils::parse(motion_size_, "motion_size", config, false, "BEHAVIOR");

            auto com_final = com_init;
            com_final(2) -= motion_size_;

            trajectories_.push_back(trajectory_handler::compute_traj(com_init, com_final, controller_->dt(), trajectory_duration_));
            trajectories_.push_back(trajectory_handler::compute_traj(com_final, com_init, controller_->dt(), trajectory_duration_));
            current_trajectory_ = trajectories_[traj_selector_];
        }

        bool TalosSquat::update()
        {
            auto ref = current_trajectory_[time_];
            std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_com_ref(ref);
            if (controller_->solve()) {
                time_++;
                if (time_ == current_trajectory_.size()) {
                    time_ = 0;
                    traj_selector_ = ++traj_selector_ % trajectories_.size();
                    current_trajectory_ = trajectories_[traj_selector_];
                }
                return true;
            }
            else {
                return false;
            }
        }
    } // namespace behaviors
} // namespace inria_wbc
