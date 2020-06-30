#include "examples/talos_squat.hpp"

namespace tsid_sot
{
    namespace example
    {

        static AutoRegister<TalosSquat> __talos_squat("talos-squat");

        TalosSquat::TalosSquat(const tsid_sot::controllers::TalosBaseController::Params &params)
        {
            //////////////////// INIT STACK OF TASK //////////////////////////////////////
            controller_ = std::make_shared<tsid_sot::controllers::TalosPosTracking>(params);

            //////////////////// DEFINE COM TRAJECTORIES  //////////////////////////////////////
            traj_selector_ = 0;
            auto com_init = std::static_pointer_cast<tsid_sot::controllers::TalosPosTracking>(controller_)->com_init();
            auto com_final = com_init;
            com_final(2) -= 0.2;
            float trajectory_duration = 3;
            trajectories_.push_back(trajectory_handler::compute_traj(com_init, com_final, params.dt, trajectory_duration));
            trajectories_.push_back(trajectory_handler::compute_traj(com_final, com_init, params.dt, trajectory_duration));
            current_trajectory_ = trajectories_[traj_selector_];
        }

        Eigen::VectorXd TalosSquat::cmd()
        {

            auto ref = current_trajectory_[time_];
            std::static_pointer_cast<tsid_sot::controllers::TalosPosTracking>(controller_)->set_com_ref(ref);
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
