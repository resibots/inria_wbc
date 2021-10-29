#include "inria_wbc/behaviors/humanoid/walk.hpp"

namespace inria_wbc {
    namespace behaviors {
        namespace humanoid {
            static Register<Walk> __walk("humanoid::walk");

            Walk::Walk(const controller_ptr_t& controller, const YAML::Node& config) : Behavior(controller, config)
            {
                auto c = IWBC_CHECK(config["BEHAVIOR"]);
                traj_com_duration_ = IWBC_CHECK(c["traj_com_duration"].as<float>());
                traj_foot_duration_ = IWBC_CHECK(c["traj_foot_duration"].as<float>());
                step_height_ = IWBC_CHECK(c["step_height"].as<float>());
                step_length_ = IWBC_CHECK(c["step_length"].as<float>());
                num_of_cycles_ = IWBC_CHECK(c["num_of_cycles"].as<int>());

                behavior_type_ = this->behavior_type();
                controller_->set_behavior_type(behavior_type_);

                dt_ = controller_->dt();

                state_ = States::INIT;
                time_ = 0;
                _generate_trajectories(num_of_cycles_);
            }

            void Walk::_generate_trajectories(int num_of_cycles)
            {
                std::vector<States> cycle_to_repeat = {
                    States::LIFT_DOWN_LF,
                    States::MOVE_COM_LEFT,
                    States::LIFT_UP_RF,
                    States::LIFT_DOWN_RF,
                    States::MOVE_COM_RIGHT,
                    States::LIFT_UP_LF};

                cycle_.push_back(States::INIT);
                cycle_.push_back(States::LF_INIT);

                for (int i = 0; i < num_of_cycles; i++) {
                    for (auto& c : cycle_to_repeat) {
                        cycle_.push_back(c);
                    }
                }

                auto controller = std::dynamic_pointer_cast<inria_wbc::controllers::TalosPosTracker>(controller_);
                assert(controller);
                auto translate = [](const pinocchio::SE3& p, double v, int index) {
                    auto p2 = p;
                    p2.translation()(index) += v;
                    return p2;
                };
                // set the waypoints for the feet
                auto lf_low = controller->model_joint_pos("leg_left_6_joint");
                auto lf_high = translate(lf_low, step_height_, 2);

                auto rf_low = controller->model_joint_pos("leg_right_6_joint");
                auto rf_high = translate(rf_low, step_height_, 2);

                // set the waypoints for the CoM : lf/rf but same height
                Eigen::VectorXd com_init = controller->get_com_ref();
                Eigen::VectorXd com_lf = lf_low.translation();
                com_lf(2) = com_init(2);
                Eigen::VectorXd com_rf = rf_low.translation();
                com_rf(2) = com_init(2);

                auto lh_init = controller->get_se3_ref("lh");
                auto rh_init = controller->get_se3_ref("rh");
                auto lh_forward = controller->get_se3_ref("lh");
                auto rh_forward = controller->get_se3_ref("rh");
                float diff = 0.0;
                // we do this so that we can simply alter the cycle_ vector (like, not lifting feet)
                for (auto c : cycle_) {
                    switch (c) {
                    case States::INIT:
                        _rf_trajs.push_back(trajectory_handler::constant_traj(rf_low, dt_, traj_com_duration_));
                        _lf_trajs.push_back(trajectory_handler::constant_traj(lf_low, dt_, traj_com_duration_));
                        _com_trajs.push_back(trajectory_handler::compute_traj(com_init, com_rf, dt_, traj_com_duration_));
                        _lh_trajs.push_back(trajectory_handler::constant_traj(lh_init, dt_, traj_com_duration_));
                        _rh_trajs.push_back(trajectory_handler::constant_traj(rh_init, dt_, traj_com_duration_));
                        break;
                    case States::LF_INIT:
                        _rf_trajs.push_back(trajectory_handler::constant_traj(rf_low, dt_, traj_foot_duration_));
                        _lf_trajs.push_back(trajectory_handler::compute_traj(lf_low, lf_high, dt_, traj_foot_duration_));
                        _com_trajs.push_back(trajectory_handler::constant_traj(com_rf, dt_, traj_foot_duration_));
                        _lh_trajs.push_back(trajectory_handler::constant_traj(lh_init, dt_, traj_foot_duration_));
                        _rh_trajs.push_back(trajectory_handler::constant_traj(rh_init, dt_, traj_foot_duration_));
                        break;
                    case States::LIFT_DOWN_LF:
                        diff = rf_low.translation()(0) - lf_low.translation()(0);
                        // lf_low.translation()(0) = rf_low.translation()(0);
                        lf_low = translate(lf_low, diff + step_length_, 0);
                        lh_forward = translate(lh_init, diff + step_length_, 0);
                        rh_forward = translate(rh_init, diff + step_length_, 0);
                        _rf_trajs.push_back(trajectory_handler::constant_traj(rf_low, dt_, traj_foot_duration_));
                        _lf_trajs.push_back(trajectory_handler::compute_traj(lf_high, lf_low, dt_, traj_foot_duration_));
                        _com_trajs.push_back(trajectory_handler::constant_traj(com_rf, dt_, traj_foot_duration_));
                        _lh_trajs.push_back(trajectory_handler::compute_traj(lh_init, lh_forward, dt_, traj_com_duration_));
                        _rh_trajs.push_back(trajectory_handler::compute_traj(rh_init, rh_forward, dt_, traj_com_duration_));
                        lh_init = lh_forward;
                        rh_init = rh_forward;
                        break;
                    case States::MOVE_COM_LEFT:
                        com_lf(0) = lf_low.translation()(0);
                        _rf_trajs.push_back(trajectory_handler::constant_traj(rf_low, dt_, traj_com_duration_));
                        _lf_trajs.push_back(trajectory_handler::constant_traj(lf_low, dt_, traj_com_duration_));
                        _com_trajs.push_back(trajectory_handler::compute_traj(com_rf, com_lf, dt_, traj_com_duration_));
                        _lh_trajs.push_back(trajectory_handler::constant_traj(lh_init, dt_, traj_com_duration_));
                        _rh_trajs.push_back(trajectory_handler::constant_traj(rh_init, dt_, traj_com_duration_));
                        break;
                    case States::LIFT_UP_RF:
                        rf_high.translation()(0) = lf_low.translation()(0);
                        _rf_trajs.push_back(trajectory_handler::compute_traj(rf_low, rf_high, dt_, traj_foot_duration_));
                        _lf_trajs.push_back(trajectory_handler::constant_traj(lf_low, dt_, traj_foot_duration_));
                        _com_trajs.push_back(trajectory_handler::constant_traj(com_lf, dt_, traj_foot_duration_));
                        _lh_trajs.push_back(trajectory_handler::constant_traj(lh_init, dt_, traj_com_duration_));
                        _rh_trajs.push_back(trajectory_handler::constant_traj(rh_init, dt_, traj_com_duration_));
                        break;
                    case States::LIFT_DOWN_RF:
                        rf_low = translate(rf_low, 2 * step_length_, 0);
                        _rf_trajs.push_back(trajectory_handler::compute_traj(rf_high, rf_low, dt_, traj_foot_duration_));
                        _lf_trajs.push_back(trajectory_handler::constant_traj(lf_low, dt_, traj_foot_duration_));
                        _com_trajs.push_back(trajectory_handler::constant_traj(com_lf, dt_, traj_foot_duration_));
                        _lh_trajs.push_back(trajectory_handler::constant_traj(lh_init, dt_, traj_com_duration_));
                        _rh_trajs.push_back(trajectory_handler::constant_traj(rh_init, dt_, traj_com_duration_));
                        break;
                    case States::MOVE_COM_RIGHT:
                        com_rf(0) = rf_low.translation()(0);
                        _rf_trajs.push_back(trajectory_handler::constant_traj(rf_low, dt_, traj_com_duration_));
                        _lf_trajs.push_back(trajectory_handler::constant_traj(lf_low, dt_, traj_com_duration_));
                        _com_trajs.push_back(trajectory_handler::compute_traj(com_lf, com_rf, dt_, traj_com_duration_));
                        _lh_trajs.push_back(trajectory_handler::constant_traj(lh_init, dt_, traj_com_duration_));
                        _rh_trajs.push_back(trajectory_handler::constant_traj(rh_init, dt_, traj_com_duration_));
                        break;
                    case States::LIFT_UP_LF:
                        lf_high.translation()(0) = rf_low.translation()(0);
                        _rf_trajs.push_back(trajectory_handler::constant_traj(rf_low, dt_, traj_foot_duration_));
                        _lf_trajs.push_back(trajectory_handler::compute_traj(lf_low, lf_high, dt_, traj_foot_duration_));
                        _com_trajs.push_back(trajectory_handler::constant_traj(com_rf, dt_, traj_foot_duration_));
                        _lh_trajs.push_back(trajectory_handler::constant_traj(lh_init, dt_, traj_com_duration_));
                        _rh_trajs.push_back(trajectory_handler::constant_traj(rh_init, dt_, traj_com_duration_));
                        break;
                    default:
                        assert(0 && "unknown state");
                    }
                }
                assert(_rf_trajs.size() == cycle_.size());
                assert(_lf_trajs.size() == cycle_.size());
                assert(_com_trajs.size() == cycle_.size());
                assert(_lh_trajs.size() == cycle_.size());
                assert(_rh_trajs.size() == cycle_.size());
            }

            void Walk::update(const controllers::SensorData& sensor_data)
            {

                auto controller = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracker>(controller_);

                if (run_) {
                    // add and remove contacts
                    if (time_ == 0 && (state_ == States::LIFT_UP_LF || state_ == States::LF_INIT))
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
                    controller->set_contact_se3_ref(_lf_trajs[_current_traj][time_], "contact_lfoot");
                    controller->set_contact_se3_ref(_rf_trajs[_current_traj][time_], "contact_rfoot");
                    controller->set_se3_ref(_lh_trajs[_current_traj][time_], "lh");
                    controller->set_se3_ref(_rh_trajs[_current_traj][time_], "rh");
                    if (time_ == 0) {
                        std::cout << "state " << state_ << " curr_traj " << _current_traj << std::endl;
                        std::cout << "INIT " << std::endl;
                        std::cout << "lh " << _lh_trajs[_current_traj][time_].translation().transpose() << std::endl;
                        std::cout << "rh " << _rh_trajs[_current_traj][time_].translation().transpose() << std::endl;
                        std::cout << "lf " << _lf_trajs[_current_traj][time_].translation().transpose() << std::endl;
                        std::cout << "rf " << _rf_trajs[_current_traj][time_].translation().transpose() << std::endl;
                        std::cout << "com " << _com_trajs[_current_traj][time_].transpose() << std::endl;
                        std::cout << "DEST " << std::endl;
                        std::cout << "lh " << _lh_trajs[_current_traj].back().translation().transpose() << std::endl;
                        std::cout << "rh " << _rh_trajs[_current_traj].back().translation().transpose() << std::endl;
                        std::cout << "lf " << _lf_trajs[_current_traj].back().translation().transpose() << std::endl;
                        std::cout << "rf " << _rf_trajs[_current_traj].back().translation().transpose() << std::endl;
                        std::cout << "com " << _com_trajs[_current_traj].back().transpose() << std::endl;
                    }
                }

                controller_->update(sensor_data);

                if (run_) {
                    time_++;
                    if (time_ == _com_trajs[_current_traj].size()) {
                        time_ = 0;
                        _current_traj = ++_current_traj;

                        if (_current_traj < cycle_.size()) {
                            state_ = cycle_[_current_traj];
                        }
                        else {
                            run_ = false;
                        }
                    }
                }
            }
        } // namespace humanoid
    } // namespace behaviors
} // namespace inria_wbc