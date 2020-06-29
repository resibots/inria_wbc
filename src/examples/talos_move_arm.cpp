#include "examples/talos_move_arm.hpp"

namespace tsid_sot
{
    namespace example
    {
        static AutoRegister<TalosMoveArm> __talos_move_arm("talos-move-arm");

        TalosMoveArm::TalosMoveArm(const tsid_sot::controllers::TalosBaseController::Params &params,
                                   const std::string &sot_config_path,
                                   const std::string &fb_joint_name,
                                   const std::vector<std::string> &mimic_joint_names,
                                   bool verbose)
        {
            //////////////////// INIT STACK OF TASK //////////////////////////////////////
            controller_ = std::make_shared<tsid_sot::controllers::TalosPosTracking>(params, sot_config_path, fb_joint_name, mimic_joint_names, verbose);

            //////////////////// DEFINE COM TRAJECTORIES  //////////////////////////////////////
            traj_selector_ = 0;
            auto lh_init = std::static_pointer_cast<tsid_sot::controllers::TalosPosTracking>(controller_)->lh_init();
            auto lh_final = lh_init;
            lh_final.translation()(2) += 0.05;
            float trajectory_duration = 3;
            trajectories_.push_back(trajectory_handler::compute_traj(lh_init, lh_final, params.dt, trajectory_duration));
            trajectories_.push_back(trajectory_handler::compute_traj(lh_final, lh_init, params.dt, trajectory_duration));
            current_trajectory_ = trajectories_[traj_selector_];
        }

        Eigen::VectorXd TalosMoveArm::cmd()
        {

            auto ref = current_trajectory_[time_];
            std::static_pointer_cast<tsid_sot::controllers::TalosPosTracking>(controller_)->set_lh_ref(ref);
            controller_->solve();
            time_++;
            if (time_ == current_trajectory_.size())
            {
                time_ = 0;
                traj_selector_ = ++traj_selector_ % trajectories_.size();
                current_trajectory_ = trajectories_[traj_selector_];
            }

            return controller_->q(false);
        }

    } // namespace example
} // namespace tsid_sot
