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

            YAML::Node config = YAML::LoadFile(controller_->params().sot_config_path);
            inria_wbc::utils::parse(trajectory_duration_, "trajectory_duration", config, false, "BEHAVIOR");
            inria_wbc::utils::parse(motion_size_, "motion_size", config, false, "BEHAVIOR");

            state_ = States::MOVE_COM_RIGHT;
            dt_ = params.dt;
        }

        bool WalkOnSpot::update()
        {
            switch (state_)
            {
            case States::MOVE_COM_RIGHT:
                if(first_run_){
                    std::cout << "Move CoM to right foot" << std::endl;
                    first_run_ = false;
                    auto com_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_pinocchio_com();
                    rf_init_ = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_RF_SE3();
                    auto right_foot_pos = rf_init_.translation();
                    Eigen::VectorXd com_right = com_init;
                    com_right(0) = right_foot_pos(0);
                    com_right(1) = right_foot_pos(1);
                    current_com_trajectory_ = trajectory_handler::compute_traj(com_init, com_right, dt_, trajectory_duration_);
                }
                break;
            case States::LIFT_UP_LF:
                if(first_run_){
                    std::cout << "Lift up left foot" << std::endl;
                    first_run_ = false;
                    auto com_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_pinocchio_com();
                    current_com_trajectory_ = trajectory_handler::compute_traj(com_init, com_init, dt_, trajectory_duration_);
                    std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->remove_contact("contact_lfoot");
                    lf_init_ = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_LF_SE3();
                    auto left_foot_pos = lf_init_.translation();
                    left_foot_pos(2) += 0.1;
                    pinocchio::SE3 lf_final(lf_init_.rotation(),left_foot_pos);
                    current_foot_trajectory_ = trajectory_handler::compute_traj(lf_init_, lf_final, dt_, trajectory_duration_);
                }
                std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_se3_ref(current_foot_trajectory_[time_], "lf");
                break;
            case States::LIFT_DOWN_LF:
                if(first_run_){
                    std::cout << "Lift down left foot" << std::endl;
                    first_run_ = false;
                    auto com_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_pinocchio_com();
                    current_com_trajectory_ = trajectory_handler::compute_traj(com_init, com_init, dt_, trajectory_duration_);

                    lf_init_ = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_LF_SE3();
                    auto left_foot_pos = lf_init_.translation();
                    left_foot_pos(2) -= 0.1;
                    pinocchio::SE3 lf_final(lf_init_.rotation(),left_foot_pos);
                    current_foot_trajectory_ = trajectory_handler::compute_traj(lf_init_, lf_final, dt_, trajectory_duration_);
                }
                std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_se3_ref(current_foot_trajectory_[time_], "lf");
                if (time_ == current_com_trajectory_.size()-1)
                {
                    std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->add_contact("contact_lfoot");
                }
                break;
            case States::MOVE_COM_LEFT:
                if(first_run_){
                    std::cout << "Move CoM to left foot" << std::endl;
                    first_run_ = false;
                    
                    auto com_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_pinocchio_com();
                    lf_init_ = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_LF_SE3();
                    auto left_foot_pos = lf_init_.translation();
                    Eigen::VectorXd com_left = com_init;
                    com_left(0) = left_foot_pos(0);
                    com_left(1) = left_foot_pos(1);
                    current_com_trajectory_ = trajectory_handler::compute_traj(com_init, com_left, dt_, trajectory_duration_);
                }
                break;
            case States::LIFT_UP_RF:
                if(first_run_){
                    std::cout << "Lift up right foot" << std::endl;
                    first_run_ = false;
                    auto com_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_pinocchio_com();
                    current_com_trajectory_ = trajectory_handler::compute_traj(com_init, com_init, dt_, trajectory_duration_);
                    std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->remove_contact("contact_rfoot");
                    rf_init_ = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_RF_SE3();
                    auto right_foot_pos = rf_init_.translation();
                    right_foot_pos(2) += 0.1;
                    pinocchio::SE3 rf_final(rf_init_.rotation(),right_foot_pos);
                    current_foot_trajectory_ = trajectory_handler::compute_traj(rf_init_, rf_final, dt_, trajectory_duration_);
                }
                std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_se3_ref(current_foot_trajectory_[time_], "rf");
                break;
            case States::LIFT_DOWN_RF:
                if(first_run_){
                    std::cout << "Lift down right foot" << std::endl;
                    first_run_ = false;
                    auto com_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_pinocchio_com();
                    current_com_trajectory_ = trajectory_handler::compute_traj(com_init, com_init, dt_, trajectory_duration_);

                    rf_init_ = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_RF_SE3();
                    auto right_foot_pos = rf_init_.translation();
                    right_foot_pos(2) -= 0.1;
                    pinocchio::SE3 rf_final(rf_init_.rotation(),right_foot_pos);
                    current_foot_trajectory_ = trajectory_handler::compute_traj(rf_init_, rf_final, dt_, trajectory_duration_);
                }
                std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_se3_ref(current_foot_trajectory_[time_], "rf");
                if (time_ == current_com_trajectory_.size()-1)
                {
                    std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->add_contact("contact_rfoot");
                }
                break;
            default:
                break;
            }
            auto ref = current_com_trajectory_[time_];
            std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_com_ref(ref);
            if (controller_->solve())
            {
                time_++;
                if (time_ == current_com_trajectory_.size())
                {
                    time_ = 0;
                    first_run_ = true;
                    traj_selector_ = ++traj_selector_ % cycle_size_;
                    state_ = cycle_[traj_selector_];
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
