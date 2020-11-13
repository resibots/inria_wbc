#include "inria_wbc/behaviors/talos_move_arm.hpp"

namespace inria_wbc {
    namespace behaviors {
        static Register<TalosMoveArm> __talos_move_arm("talos-move-arm");

        TalosMoveArm::TalosMoveArm(const controller_ptr_t& controller) : Behavior(controller)
        {

            //////////////////// DEFINE COM TRAJECTORIES  //////////////////////////////////////
            traj_selector_ = 0;
            auto lh_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_se3_ref("lh");

            YAML::Node config = YAML::LoadFile(controller_->params().sot_config_path);
            inria_wbc::utils::parse(trajectory_duration_, "trajectory_duration", config, "BEHAVIOR", controller_->params().verbose);
            inria_wbc::utils::parse(motion_size_, "motion_size", config, "BEHAVIOR", controller_->params().verbose);

            auto lh_final = lh_init;
            lh_final.translation()(2) += motion_size_;

            trajectories_.push_back(trajectory_handler::compute_traj(lh_init, lh_final, controller_->dt(), trajectory_duration_));
            trajectories_.push_back(trajectory_handler::compute_traj(lh_final, lh_init, controller_->dt(), trajectory_duration_));
            current_trajectory_ = trajectories_[traj_selector_];
        }

        void TalosMoveArm::update()
        {
            auto ref = current_trajectory_[time_];
            std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_se3_ref(ref, "lh");

            controller_->solve();
            time_++;
            if (time_ == current_trajectory_.size()) {
                time_ = 0;
                traj_selector_ = ++traj_selector_ % trajectories_.size();
                current_trajectory_ = trajectories_[traj_selector_];
            }
        }
    } // namespace behaviors
} // namespace inria_wbc