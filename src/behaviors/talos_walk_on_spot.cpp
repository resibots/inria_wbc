#include "inria_wbc/behaviors/talos_walk_on_spot.hpp"

namespace inria_wbc {
    namespace behaviors {

        static AutoRegister<WalkOnSpot> __talos_walk_on_spot("walk-on-spot");
        ;

        WalkOnSpot::WalkOnSpot(const inria_wbc::controllers::TalosBaseController::Params& params) : Behavior(std::make_shared<inria_wbc::controllers::TalosPosTracking>(params))
        {
            traj_selector_ = 0;

            YAML::Node config = YAML::LoadFile(controller_->params().sot_config_path);
            inria_wbc::utils::parse(trajectory_duration_, "trajectory_duration", config, false, "BEHAVIOR");
            inria_wbc::utils::parse(motion_size_, "motion_size", config, false, "BEHAVIOR");

            dt_ = params.dt;

            cycle_ = {
                States::MOVE_COM_RIGHT,
                States::LIFT_UP_LF,
                States::LIFT_DOWN_LF,
                States::MOVE_COM_LEFT,
                States::LIFT_UP_RF,
                States::LIFT_DOWN_RF};
            state_ = States::MOVE_COM_RIGHT;
        }

        bool WalkOnSpot::update()
        {
            switch (state_) {
            case States::MOVE_COM_RIGHT:
                if (time_ == 0) {
                    std::cout << "Move CoM to right foot" << std::endl;
                    // Compute current CoM trajectory to track -> move above the right foot
                    auto com_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_pinocchio_com(); // get current CoM position
                    Eigen::VectorXd com_right = com_init;
                    auto rf_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_RF_SE3(); // get current right foot position
                    auto right_foot_pos = rf_init.translation();
                    com_right(0) = right_foot_pos(0);
                    com_right(1) = right_foot_pos(1);
                    current_com_trajectory_ = trajectory_handler::compute_traj(com_init, com_right, dt_, trajectory_duration_);
                }
                break;
            case States::LIFT_UP_LF:
                if (time_ == 0) {
                    std::cout << "Lift up left foot" << std::endl;
                    // Compute current CoM trajectory to track -> stay still
                    auto com_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_pinocchio_com();
                    current_com_trajectory_ = trajectory_handler::compute_traj(com_init, com_init, dt_, trajectory_duration_);
                    // Remove left foot contact with the ground
                    std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->remove_contact("contact_lfoot");
                    // Compute current Left foot trajectory to track -> Lift up 10cm
                    auto lf_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_LF_SE3();
                    auto left_foot_pos = lf_init.translation();
                    left_foot_pos(2) += 0.1;
                    pinocchio::SE3 lf_final(lf_init.rotation(), left_foot_pos);
                    current_foot_trajectory_ = trajectory_handler::compute_traj(lf_init, lf_final, dt_, trajectory_duration_);
                }
                // add left foot task
                std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_se3_ref(current_foot_trajectory_[time_], "lf");
                break;
            case States::LIFT_DOWN_LF:
                if (time_ == 0) {
                    std::cout << "Lift down left foot" << std::endl;
                    // Compute current CoM trajectory to track -> stay still
                    auto com_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_pinocchio_com();
                    current_com_trajectory_ = trajectory_handler::compute_traj(com_init, com_init, dt_, trajectory_duration_);
                    // Compute current Left foot trajectory to track -> Lift down 10cm
                    auto lf_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_LF_SE3();
                    auto left_foot_pos = lf_init.translation();
                    left_foot_pos(2) -= 0.1;
                    pinocchio::SE3 lf_final(lf_init.rotation(), left_foot_pos);
                    current_foot_trajectory_ = trajectory_handler::compute_traj(lf_init, lf_final, dt_, trajectory_duration_);
                }
                // add left foot task
                std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_se3_ref(current_foot_trajectory_[time_], "lf");
                // add left foot contact at the end of the lifting down
                if (time_ == current_com_trajectory_.size() - 1) {
                    std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->add_contact("contact_lfoot");
                }
                break;
            case States::MOVE_COM_LEFT:
                if (time_ == 0) {
                    std::cout << "Move CoM to left foot" << std::endl;
                    // Compute current CoM trajectory to track -> move above the left foot
                    auto com_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_pinocchio_com();
                    Eigen::VectorXd com_left = com_init;
                    auto lf_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_LF_SE3();
                    auto left_foot_pos = lf_init.translation();
                    com_left(0) = left_foot_pos(0);
                    com_left(1) = left_foot_pos(1);
                    current_com_trajectory_ = trajectory_handler::compute_traj(com_init, com_left, dt_, trajectory_duration_);
                }
                break;
            case States::LIFT_UP_RF:
                if (time_ == 0) {
                    std::cout << "Lift up right foot" << std::endl;
                    // Compute current CoM trajectory to track -> stay still
                    auto com_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_pinocchio_com();
                    current_com_trajectory_ = trajectory_handler::compute_traj(com_init, com_init, dt_, trajectory_duration_);
                    // Remove right foot contact
                    std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->remove_contact("contact_rfoot");
                    // Compute current right foot trajectory to track -> Lift up 10cm
                    auto rf_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_RF_SE3();
                    auto right_foot_pos = rf_init.translation();
                    right_foot_pos(2) += 0.1;
                    pinocchio::SE3 rf_up(rf_init.rotation(), right_foot_pos);
                    current_foot_trajectory_ = trajectory_handler::compute_traj(rf_init, rf_up, dt_, trajectory_duration_);
                }
                // add right foot task
                std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_se3_ref(current_foot_trajectory_[time_], "rf");
                break;
            case States::LIFT_DOWN_RF:
                if (time_ == 0) {
                    std::cout << "Lift down right foot" << std::endl;
                    // Compute current CoM trajectory to track -> stay still
                    auto com_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_pinocchio_com();
                    current_com_trajectory_ = trajectory_handler::compute_traj(com_init, com_init, dt_, trajectory_duration_);
                    // Compute current right foot trajectory to track -> Lift down 10cm
                    auto rf_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_RF_SE3();
                    auto right_foot_pos = rf_init.translation();
                    right_foot_pos(2) -= 0.1;
                    pinocchio::SE3 rf_down(rf_init.rotation(), right_foot_pos);
                    current_foot_trajectory_ = trajectory_handler::compute_traj(rf_init, rf_down, dt_, trajectory_duration_);
                }
                // add right foot task
                std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_se3_ref(current_foot_trajectory_[time_], "rf");
                // add right foot contact at the end of the lifting down
                if (time_ == current_com_trajectory_.size() - 1) {
                    std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->add_contact("contact_rfoot");
                }
                break;
            default:
                break;
            }
            auto ref = current_com_trajectory_[time_];
            std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_com_ref(ref);
            if (controller_->solve()) {
                time_++;
                if (time_ == current_com_trajectory_.size()) {
                    time_ = 0;
                    traj_selector_ = ++traj_selector_ % cycle_.size();
                    state_ = cycle_[traj_selector_];
                }
                return true;
            }
            else {
                return false;
            }
        }

    } // namespace behaviors
} // namespace inria_wbc
