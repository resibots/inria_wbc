#include "inria_wbc/behaviors/talos_walk_on_spot.hpp"

namespace inria_wbc
{
    namespace behaviors
    {

        static AutoRegister<WalkOnSpot> __talos_squat("walk-on-spot");;
         
        WalkOnSpot::WalkOnSpot(const inria_wbc::controllers::TalosBaseController::Params &params) :
            Behavior(std::make_shared<inria_wbc::controllers::TalosPosTracking>(params))
        {
            //////////////////// DEFINE COM TRAJECTORIES  //////////////////////////////////////
            traj_selector_ = 0;
            auto com_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->com_init();

            YAML::Node config = YAML::LoadFile(controller_->params().sot_config_path);
            inria_wbc::utils::parse(trajectory_duration_, "trajectory_duration", config, false, "BEHAVIOR");
            inria_wbc::utils::parse(motion_size_, "motion_size", config, false, "BEHAVIOR");

            auto com_right = com_init;
            auto com_left = com_init;
            com_right(1) -= motion_size_;
            com_left(1) += motion_size_;


            trajectories_.push_back(trajectory_handler::compute_traj(com_init, com_right, params.dt, trajectory_duration_));
            trajectories_.push_back(trajectory_handler::compute_traj(com_right, com_right, params.dt, 3*trajectory_duration_));
            //trajectories_.push_back(trajectory_handler::compute_traj(com_left, com_init, params.dt, trajectory_duration_));

            current_trajectory_ = trajectories_[traj_selector_];
        }

        bool WalkOnSpot::update()
        {
            auto ref = current_trajectory_[time_];
            std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_com_ref(ref);
/*             if ( time_ == 5000){
                std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->removeRigidContact("contact_lfoot");
            } */
            if (controller_->solve())
            {
                time_++;
                if (time_ == current_trajectory_.size())
                {
                    time_ = 0;
                    traj_selector_ = ++traj_selector_ % trajectories_.size();
                    current_trajectory_ = trajectories_[traj_selector_];
                }
                return true;
            }
            else
            {
                return false;
            }
        }

    } // namespace behaviors
} // namespace inria_wbc
