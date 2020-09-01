#include "inria_wbc/behaviors/talos_squat.hpp"

namespace inria_wbc
{
    namespace behaviors
    {

        static AutoRegister<TalosSquat> __talos_squat("talos-squat");

        TalosSquat::TalosSquat(const inria_wbc::controllers::TalosBaseController::Params &params) :
            Behavior(std::make_shared<inria_wbc::controllers::TalosPosTracking>(params))
        {
            //////////////////// DEFINE COM TRAJECTORIES  //////////////////////////////////////
            traj_selector_ = 0;
            auto com_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->com_init();
            auto com_final = com_init;
            com_final(2) -= 0.2;
            float trajectory_duration = 3;
            trajectories_.push_back(trajectory_handler::compute_traj(com_init, com_final, params.dt, trajectory_duration));
            trajectories_.push_back(trajectory_handler::compute_traj(com_final, com_init, params.dt, trajectory_duration));
            current_trajectory_ = trajectories_[traj_selector_];
        }

        bool TalosSquat::update()
        {
            auto ref = current_trajectory_[time_];
            std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_com_ref(ref);
            if (controller_->solve())
            {
                time_++;
                if (time_ == current_trajectory_.size())
                {
                    time_ = 0;
                    traj_selector_ = ++traj_selector_ % trajectories_.size();
                    current_trajectory_ = trajectories_[traj_selector_];
                }
                // q.resize(controller_->q(false).size());
                // q = controller_->q(false);
                return true;
            }
            else
            {
                return false;
            }
        }
    } // namespace behaviors
} // namespace inria_wbc
