#include "inria_wbc/behaviors/talos_walk_on_spot.hpp"
#include <chrono>

namespace inria_wbc {
    namespace behaviors {

        static Register<WalkOnSpot> __talos_walk_on_spot("walk-on-spot");

        WalkOnSpot::WalkOnSpot(const controller_ptr_t& controller) : Behavior(controller)
        {
            YAML::Node config = YAML::LoadFile(controller_->params().sot_config_path);
            inria_wbc::utils::parse(traj_foot_duration_, "traj_foot_duration", config, false, "BEHAVIOR"); // TODO use the verbose
            inria_wbc::utils::parse(traj_com_duration_, "traj_com_duration", config, false, "BEHAVIOR");
            inria_wbc::utils::parse(step_height_, "step_height", config, false, "BEHAVIOR");

            dt_ = controller_->dt();

            state_ = States::INIT;
            time_ = 0;
            _generate_trajectories();
        }

        void WalkOnSpot::_generate_trajectories()
        {
            cycle_ = {
                States::INIT, // will be skipped
                States::LIFT_UP_LF,
                States::LIFT_DOWN_LF,
                States::MOVE_COM_LEFT,
                States::LIFT_UP_RF,
                States::LIFT_DOWN_RF,
                States::MOVE_COM_RIGHT};

            auto controller = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_);
            auto translate_up = [](const pinocchio::SE3& p, double v) {
                auto p2 = p;
                p2.translation()(2) += v;
                return p2;
            };
            // set the waypoints for the feet
            Eigen::VectorXd high = (Eigen::VectorXd(3) << 0, step_height_, 0).finished();
            auto lf_low = controller->get_se3_ref("lf");
            auto lf_high = translate_up(lf_low, step_height_);

            auto rf_low = controller->get_se3_ref("rf");
            auto rf_high = translate_up(rf_low, step_height_);

            // set the waypoints for the CoM : lf/rf but same height
            Eigen::VectorXd com_init = controller->get_pinocchio_com();
            Eigen::VectorXd com_lf = lf_low.translation();
            com_lf(2) = com_init(2);
            Eigen::VectorXd com_rf = rf_low.translation();
            com_rf(2) = com_init(2);

            // we do this so that we can simply alter the cycle_ vector (like, not lifting feet)
            for (auto c : cycle_) {
                switch (c) {
                case States::INIT:
                    _rf_trajs.push_back(trajectory_handler::constant_traj(rf_low, dt_, traj_com_duration_));
                    _lf_trajs.push_back(trajectory_handler::constant_traj(lf_low, dt_, traj_com_duration_));
                    _com_trajs.push_back(trajectory_handler::compute_traj(com_init, com_rf, dt_, traj_com_duration_));
                    break;
                case States::LIFT_UP_LF:
                    _rf_trajs.push_back(trajectory_handler::constant_traj(rf_low, dt_, traj_foot_duration_));
                    _lf_trajs.push_back(trajectory_handler::compute_traj(lf_low, lf_high, dt_, traj_foot_duration_));
                    _com_trajs.push_back(trajectory_handler::constant_traj(com_rf, dt_, traj_foot_duration_));
                    break;
                case States::LIFT_DOWN_LF:
                    _rf_trajs.push_back(trajectory_handler::constant_traj(rf_low, dt_, traj_foot_duration_));
                    _lf_trajs.push_back(trajectory_handler::compute_traj(lf_high, lf_low, dt_, traj_foot_duration_));
                    _com_trajs.push_back(trajectory_handler::constant_traj(com_rf, dt_, traj_foot_duration_));
                    break;
                case States::MOVE_COM_LEFT:
                    _rf_trajs.push_back(trajectory_handler::constant_traj(rf_low, dt_, traj_com_duration_));
                    _lf_trajs.push_back(trajectory_handler::constant_traj(lf_low, dt_, traj_com_duration_));
                    _com_trajs.push_back(trajectory_handler::compute_traj(com_rf, com_lf, dt_, traj_com_duration_));
                    break;
                case States::LIFT_UP_RF:
                    _rf_trajs.push_back(trajectory_handler::compute_traj(rf_low, rf_high, dt_, traj_foot_duration_));
                    _lf_trajs.push_back(trajectory_handler::constant_traj(lf_high, dt_, traj_foot_duration_));
                    _com_trajs.push_back(trajectory_handler::constant_traj(com_lf, dt_, traj_foot_duration_));
                    break;
                case States::LIFT_DOWN_RF:
                    _rf_trajs.push_back(trajectory_handler::compute_traj(rf_high, rf_low, dt_, traj_foot_duration_));
                    _lf_trajs.push_back(trajectory_handler::constant_traj(lf_low, dt_, traj_foot_duration_));
                    _com_trajs.push_back(trajectory_handler::constant_traj(com_lf, dt_, traj_foot_duration_));
                    break;
                case States::MOVE_COM_RIGHT:
                    _rf_trajs.push_back(trajectory_handler::constant_traj(rf_low, dt_, traj_com_duration_));
                    _lf_trajs.push_back(trajectory_handler::constant_traj(lf_low, dt_, traj_com_duration_));
                    _com_trajs.push_back(trajectory_handler::compute_traj(com_lf, com_rf, dt_, traj_com_duration_));
                    break;
                default:
                    assert(0 && "unknown state");
                }
            }
            assert(_rf_trajs.size() == cycle_.size());
            assert(_lf_trajs.size() == cycle_.size());
            assert(_com_trajs.size() == cycle_.size());
        }

        bool WalkOnSpot::update()
        {
            auto controller = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_);
            auto t1_traj = std::chrono::high_resolution_clock::now();

            // add and remove contacts
            if (time_ == 0 && state_ == States::LIFT_UP_LF)
                controller->remove_contact("contact_lfoot");
            if (time_ == 0 && state_ == States::LIFT_UP_RF)
                controller->remove_contact("contact_rfoot");
            if (time_ == _com_trajs[_current_traj].size() - 1 && state_ == States::LIFT_DOWN_LF)
                controller->add_contact("contact_lfoot");
            if (time_ == _com_trajs[_current_traj].size() - 1 && state_ == States::LIFT_DOWN_RF)
                controller->add_contact("contact_rfoot");

            assert(time_ < _com_trajs[_current_traj].size());
            assert(time_ < _rf_trajs[_current_traj].size());
            assert(time_ < _lf_trajs[_current_traj].size());

            controller->set_com_ref(_com_trajs[_current_traj][time_]);
            controller->set_se3_ref(_lf_trajs[_current_traj][time_], "lf");
            controller->set_se3_ref(_rf_trajs[_current_traj][time_], "rf");

            auto t2_traj = std::chrono::high_resolution_clock::now();
            double step_traj = std::chrono::duration_cast<std::chrono::microseconds>(t2_traj - t1_traj).count() / 1000.0;

            if (controller_->solve()) {
                time_++;
                if (time_ == _com_trajs[_current_traj].size()) {
                    time_ = 0;
                    _current_traj = ++_current_traj % cycle_.size();
                    // we skip the init_traj
                    if (_current_traj == 0)
                        _current_traj++;
                    state_ = cycle_[_current_traj];
                }
                return true;
            }
            else {
                return false;
            }
        }

    } // namespace behaviors
} // namespace inria_wbc
