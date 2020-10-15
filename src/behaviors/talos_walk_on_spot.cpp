#include "inria_wbc/behaviors/talos_walk_on_spot.hpp"
#include <chrono>

//#define LOG_WALK_ON_SPOT
namespace inria_wbc {
    namespace behaviors {

        static AutoRegister<WalkOnSpot> __talos_walk_on_spot("walk-on-spot");

        WalkOnSpot::WalkOnSpot(const inria_wbc::controllers::TalosBaseController::Params& params) : Behavior(std::make_shared<inria_wbc::controllers::TalosPosTracking>(params))
        {
            traj_selector_ = 0;

            YAML::Node config = YAML::LoadFile(controller_->params().sot_config_path);
            inria_wbc::utils::parse(traj_foot_duration_, "traj_foot_duration", config, false, "BEHAVIOR");
            inria_wbc::utils::parse(traj_com_duration_, "traj_com_duration", config, false, "BEHAVIOR");
            inria_wbc::utils::parse(step_height_, "step_height", config, false, "BEHAVIOR");

            dt_ = params.dt;

            cycle_ = {
                States::MOVE_COM_RIGHT,
                States::LIFT_UP_LF,
                States::LIFT_DOWN_LF,
                States::MOVE_COM_LEFT,
                States::LIFT_UP_RF,
                States::LIFT_DOWN_RF};
            // cycle_ = {
            //     States::MOVE_COM_RIGHT,
            //     States::MOVE_COM_LEFT};
            state_ = States::MOVE_COM_RIGHT;
            auto controller = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_);
            _last_com = controller->get_pinocchio_com();
            _last_lf = controller->get_LF_SE3();
            _last_rf = controller->get_RF_SE3();
        }

        bool WalkOnSpot::update()
        {
            auto controller = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_);
            auto t1_traj = std::chrono::high_resolution_clock::now();

            switch (state_) {
            case States::MOVE_COM_RIGHT:
                if (time_ == 0) {
                    std::cout << "Move CoM to right foot" << std::endl;
                    // feet do not move
                    current_rf_trajectory_ = trajectory_handler::compute_traj(_last_rf, _last_rf, dt_, traj_com_duration_);
                    current_lf_trajectory_ = trajectory_handler::compute_traj(_last_lf, _last_lf, dt_, traj_com_duration_);
                    // Compute current CoM trajectory to track -> move above the right foot
                    auto com_init = _last_com;
                    Eigen::VectorXd com_right = com_init;
                    auto rf_init = _last_rf;
                    auto right_foot_pos = rf_init.translation();
                    com_right(0) = right_foot_pos(0);
                    com_right(1) = right_foot_pos(1);
                    current_com_trajectory_ = trajectory_handler::compute_traj(com_init, com_right, dt_, traj_com_duration_);
                }
                break;
            case States::LIFT_UP_LF:
                if (time_ == 0) {
                    std::cout << "Lift up left foot" << std::endl;
                    // Compute current CoM trajectory to track -> stay still
                    current_com_trajectory_ = trajectory_handler::compute_traj(_last_com, _last_com, dt_, traj_foot_duration_);
                    // rf does not move
                    current_rf_trajectory_ = trajectory_handler::compute_traj(_last_rf, _last_rf, dt_, traj_foot_duration_);
                    // Remove left foot contact with the ground
                    controller->remove_contact("contact_lfoot");
                    // Compute current Left foot trajectory to track -> Lift up 10cm
                    auto lf_init = _last_lf;
                    auto left_foot_pos = lf_init.translation();
                    left_foot_pos(2) += step_height_;
                    pinocchio::SE3 lf_final(lf_init.rotation(), left_foot_pos);
                    current_lf_trajectory_ = trajectory_handler::compute_traj(lf_init, lf_final, dt_, traj_foot_duration_);
                }
                break;
            case States::LIFT_DOWN_LF:
                if (time_ == 0) {
                    std::cout << "Lift down left foot" << std::endl;
                    // Compute current CoM trajectory to track -> stay still
                    current_com_trajectory_ = trajectory_handler::compute_traj(_last_com, _last_com, dt_, traj_foot_duration_);
                    // rf does not move
                    current_rf_trajectory_ = trajectory_handler::compute_traj(_last_rf, _last_rf, dt_, traj_foot_duration_);
                    // Compute current Left foot trajectory to track -> Lift down 10cm
                    auto lf_init = _last_lf;
                    auto left_foot_pos = lf_init.translation();
                    left_foot_pos(2) -= step_height_;
                    pinocchio::SE3 lf_final(lf_init.rotation(), left_foot_pos);
                    current_lf_trajectory_ = trajectory_handler::compute_traj(lf_init, lf_final, dt_, traj_foot_duration_);
                }
                // add left foot contact at the end of the lifting down
                if (time_ == current_com_trajectory_.size() - 1) {
                    controller->add_contact("contact_lfoot");
                }
                break;
            case States::MOVE_COM_LEFT:
                if (time_ == 0) {
                    std::cout << "Move CoM to left foot" << std::endl;
                    current_rf_trajectory_ = trajectory_handler::compute_traj(_last_rf, _last_rf, dt_, traj_com_duration_);
                    current_lf_trajectory_ = trajectory_handler::compute_traj(_last_lf, _last_lf, dt_, traj_com_duration_);
                    // Compute current CoM trajectory to track -> move above the left foot
                    auto com_init = _last_com; //controller->get_pinocchio_com();
                    Eigen::VectorXd com_left = com_init;
                    auto lf_init = _last_lf;
                    auto left_foot_pos = lf_init.translation();
                    com_left(0) = left_foot_pos(0);
                    com_left(1) = left_foot_pos(1);
                    current_com_trajectory_ = trajectory_handler::compute_traj(com_init, com_left, dt_, traj_com_duration_);
                }
                break;
            case States::LIFT_UP_RF:
                if (time_ == 0) {
                    std::cout << "Lift up right foot" << std::endl;
                    // Compute current CoM trajectory to track -> stay still
                    current_com_trajectory_ = trajectory_handler::compute_traj(_last_com, _last_com, dt_, traj_foot_duration_);
                    current_lf_trajectory_ = trajectory_handler::compute_traj(_last_lf, _last_lf, dt_, traj_foot_duration_);

                    // Remove right foot contact
                    controller->remove_contact("contact_rfoot");
                    // Compute current right foot trajectory to track -> Lift up 10cm
                    auto rf_init = _last_rf;
                    auto right_foot_pos = rf_init.translation();
                    right_foot_pos(2) += step_height_;
                    pinocchio::SE3 rf_up(rf_init.rotation(), right_foot_pos);
                    current_rf_trajectory_ = trajectory_handler::compute_traj(rf_init, rf_up, dt_, traj_foot_duration_);
                }
                break;
            case States::LIFT_DOWN_RF:
                if (time_ == 0) {
                    std::cout << "Lift down right foot" << std::endl;
                    // Compute current CoM trajectory to track -> stay still
                    auto com_init = _last_com;
                    current_com_trajectory_ = trajectory_handler::compute_traj(com_init, com_init, dt_, traj_foot_duration_);
                    current_lf_trajectory_ = trajectory_handler::compute_traj(_last_lf, _last_lf, dt_, traj_foot_duration_);
                    // Compute current right foot trajectory to track -> Lift down 10cm
                    auto rf_init = _last_rf;
                    auto right_foot_pos = rf_init.translation();
                    right_foot_pos(2) -= step_height_;
                    pinocchio::SE3 rf_down(rf_init.rotation(), right_foot_pos);
                    current_rf_trajectory_ = trajectory_handler::compute_traj(rf_init, rf_down, dt_, traj_foot_duration_);
                }
                // add right foot contact at the end of the lifting down
                if (time_ == current_com_trajectory_.size() - 1) {
                    controller->add_contact("contact_rfoot");
                }
                break;
            default:
                break;
            }
            assert(time_ < current_com_trajectory_.size());
            assert(time_ < current_lf_trajectory_.size());
            assert(time_ < current_rf_trajectory_.size());

            // follow the trajectories
            _last_com = current_com_trajectory_[time_];
            _last_lf = current_lf_trajectory_[time_];
            _last_rf = current_rf_trajectory_[time_];

            controller->set_com_ref(_last_com);
            controller->set_se3_ref(_last_lf, "lf");
            controller->set_se3_ref(_last_rf, "rf");
            auto t2_traj = std::chrono::high_resolution_clock::now();
            double step_traj = std::chrono::duration_cast<std::chrono::microseconds>(t2_traj - t1_traj).count() / 1000.0;

#ifdef LOG_WALK_ON_SPOT
            {
                static std::ofstream ofs_com("com.dat");
                static std::ofstream ofs_com_ref("com_ref.dat");
                static std::ofstream ofs_lf("lf.dat");
                static std::ofstream ofs_lf_ref("lf_ref.dat");
                static std::ofstream ofs_rf("rf.dat");
                static std::ofstream ofs_rf_ref("rf_ref.dat");
                static std::ofstream ofs_time_traj("time_traj.dat");

                ofs_com << controller->get_pinocchio_com().transpose() << std::endl;
                ofs_com_ref << _last_com.transpose() << std::endl;

                ofs_lf << controller->get_LF_SE3().translation().transpose() << std::endl;
                ofs_lf_ref << _last_lf.translation().transpose() << std::endl;

                ofs_rf << controller->get_RF_SE3().translation().transpose() << std::endl;
                ofs_rf_ref << _last_rf.translation().transpose() << std::endl;

                ofs_time_traj << step_traj << std::endl;
            }
#endif
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
