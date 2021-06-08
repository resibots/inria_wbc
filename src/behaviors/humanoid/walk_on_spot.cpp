#include "inria_wbc/behaviors/humanoid/walk_on_spot.hpp"

namespace inria_wbc {
    namespace behaviors {
        namespace humanoid {
            static Register<WalkOnSpot> __walk_on_spot("humanoid::walk-on-spot");

            WalkOnSpot::WalkOnSpot(const controller_ptr_t& controller, const YAML::Node& config) : Behavior(controller, config)
            {
                auto c = IWBC_CHECK(config["BEHAVIOR"]);
                traj_com_duration_ = IWBC_CHECK(c["traj_com_duration"].as<float>());
                traj_foot_duration_ = IWBC_CHECK(c["traj_foot_duration"].as<float>());
                step_height_ = IWBC_CHECK(c["step_height"].as<float>());
                stop_duration_ = IWBC_CHECK(c["stop_duration"].as<float>());
                stop_height_ = IWBC_CHECK(c["stop_height"].as<float>());

                behavior_type_ = this->behavior_type();
                controller_->set_behavior_type(behavior_type_);

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

                auto controller = std::dynamic_pointer_cast<inria_wbc::controllers::TalosPosTracker>(controller_);
                assert(controller);
                auto translate_up = [](const pinocchio::SE3& p, double v) {
                    auto p2 = p;
                    p2.translation()(2) += v;
                    return p2;
                };
                // set the waypoints for the feet
                Eigen::VectorXd high = (Eigen::VectorXd(3) << 0, step_height_, 0).finished();
                auto lf_low = controller->model_joint_pos("leg_left_6_joint");
                auto lf_low_stop = lf_low;
                lf_low_stop.translation()[2] += stop_height_;
                auto lf_high = translate_up(lf_low, step_height_);

                auto rf_low = controller->model_joint_pos("leg_right_6_joint");
                auto rf_low_stop = rf_low;
                rf_low_stop.translation()[2] += stop_height_;
                auto rf_high = translate_up(rf_low, step_height_);

                // set the waypoints for the CoM : lf/rf but same height
                Eigen::VectorXd com_init = controller->get_com_ref();
                Eigen::VectorXd com_lf = lf_low.translation();
                com_lf(2) = com_init(2);
                Eigen::VectorXd com_rf = rf_low.translation();
                com_rf(2) = com_init(2);

                auto append = [](auto& vect, const auto& traj) { vect.push_back(traj.front()); vect.insert(vect.end(), traj.begin(), traj.end()); };

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
                        _rf_trajs.push_back(trajectory_handler::constant_traj(rf_low, dt_, traj_foot_duration_ + stop_duration_));
                        _lf_trajs.push_back(trajectory_handler::compute_traj(lf_high, lf_low_stop, dt_, traj_foot_duration_));
                        append(_lf_trajs.back(), trajectory_handler::compute_traj(lf_low_stop, lf_low, dt_, stop_duration_));
                        assert(_lf_trajs.back().size() == _rf_trajs.back().size());
                        _com_trajs.push_back(trajectory_handler::constant_traj(com_rf, dt_, traj_foot_duration_ + stop_duration_));
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
                        _rf_trajs.push_back(trajectory_handler::compute_traj(rf_high, rf_low_stop, dt_, traj_foot_duration_));
                        append(_rf_trajs.back(), trajectory_handler::compute_traj(rf_low_stop, rf_low, dt_, stop_duration_));
                        _lf_trajs.push_back(trajectory_handler::constant_traj(lf_low, dt_, traj_foot_duration_ + stop_duration_));
                        _com_trajs.push_back(trajectory_handler::constant_traj(com_lf, dt_, traj_foot_duration_ + stop_duration_));
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

            void WalkOnSpot::update(const controllers::SensorData& sensor_data)
            {
                auto controller = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracker>(controller_);

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

                controller_->update(sensor_data);
                time_++;
                if (time_ == _com_trajs[_current_traj].size()) {
                    time_ = 0;
                    _current_traj = ++_current_traj % cycle_.size();
                    // we skip the init_traj
                    if (_current_traj == 0)
                        _current_traj++;
                    state_ = cycle_[_current_traj];
                }
            }
        } // namespace humanoid
    } // namespace behaviors
} // namespace inria_wbc