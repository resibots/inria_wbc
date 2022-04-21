#include "inria_wbc/behaviors/humanoid/walk.hpp"

namespace inria_wbc {
    namespace behaviors {
        namespace humanoid {
            static Register<Walk> __walk("humanoid::walk");

            Walk::Walk(const controller_ptr_t& controller, const YAML::Node& config) : Behavior(controller, config)
            {
                  // check that the controller is compatible
                auto h_controller = std::dynamic_pointer_cast<inria_wbc::controllers::HumanoidPosTracker>(controller_);
                IWBC_ASSERT(h_controller != NULL, "Walk: the controllers needs to be a HumanoidPosTracker (or related)!");
                IWBC_ASSERT(h_controller->has_task("lf"), "Walk: an lf task is required (left foot)");
                IWBC_ASSERT(h_controller->has_task("rf"), "Walk: an rf task is required (right foot)");
                IWBC_ASSERT(h_controller->has_task("lh"), "Walk: an lh task is required (left hand)");
                IWBC_ASSERT(h_controller->has_task("rh"), "Walk: an rh task is required (right hand)");
                IWBC_ASSERT(h_controller->has_task("com"), "Walk: a com task is required");
                IWBC_ASSERT(h_controller->has_contact("contact_lfoot"), "Walk: a contact_lfoot task is required");
                IWBC_ASSERT(h_controller->has_contact("contact_rfoot"), "Walk: a contact_rfoot task is required");

                // load the parameters
                auto c = IWBC_CHECK(config["BEHAVIOR"]);
                traj_com_duration_ = IWBC_CHECK(c["traj_com_duration"].as<float>());
                traj_foot_duration_ = IWBC_CHECK(c["traj_foot_duration"].as<float>());
                step_height_ = IWBC_CHECK(c["step_height"].as<float>());
                step_length_ = IWBC_CHECK(c["step_length"].as<float>());
                num_of_cycles_ = IWBC_CHECK(c["num_of_cycles"].as<int>());

                if (num_of_cycles_ <= 0)
                    IWBC_ERROR("num_of_cycles needs to be more than 0");

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

                cycle_.push_back(States::LIFT_DOWN_LF_FINAL);
                cycle_.push_back(States::MOVE_COM_CENTER_FINAL);

                auto controller = std::dynamic_pointer_cast<inria_wbc::controllers::HumanoidPosTracker>(controller_);
                assert(controller);
                auto translate = [](const pinocchio::SE3& p, double v, int index) {
                    auto p2 = p;
                    p2.translation()(index) += v;
                    return p2;
                };
                // set the waypoints for the feet
                auto lf_low = controller->get_se3_ref("lf");
                auto lf_high = translate(lf_low, step_height_, 2);

                auto rf_low = controller->get_se3_ref("rf");
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
                        _rf_trajs.push_back(trajs::constant_traj(rf_low, dt_, traj_com_duration_));
                        _lf_trajs.push_back(trajs::constant_traj(lf_low, dt_, traj_com_duration_));
                        _com_trajs.push_back(trajs::min_jerk_trajectory(com_init, com_rf, dt_, traj_com_duration_));
                        _lh_trajs.push_back(trajs::constant_traj(lh_init, dt_, traj_com_duration_));
                        _rh_trajs.push_back(trajs::constant_traj(rh_init, dt_, traj_com_duration_));
                        break;
                    case States::LF_INIT:
                        _rf_trajs.push_back(trajs::constant_traj(rf_low, dt_, traj_foot_duration_));
                        _lf_trajs.push_back(trajs::min_jerk_trajectory(lf_low, lf_high, dt_, traj_foot_duration_));
                        _com_trajs.push_back(trajs::constant_traj(com_rf, dt_, traj_foot_duration_));
                        _lh_trajs.push_back(trajs::constant_traj(lh_init, dt_, traj_foot_duration_));
                        _rh_trajs.push_back(trajs::constant_traj(rh_init, dt_, traj_foot_duration_));
                        break;
                    case States::LIFT_DOWN_LF:
                        diff = rf_low.translation()(0) - lf_low.translation()(0);
                        lf_low = translate(lf_low, diff + step_length_, 0);
                        lh_forward = translate(lh_init, diff + step_length_, 0);
                        _rf_trajs.push_back(trajs::constant_traj(rf_low, dt_, traj_foot_duration_));
                        _lf_trajs.push_back(trajs::min_jerk_trajectory(lf_high, lf_low, dt_, traj_foot_duration_));
                        _com_trajs.push_back(trajs::constant_traj(com_rf, dt_, traj_foot_duration_));
                        _lh_trajs.push_back(trajs::min_jerk_trajectory(lh_init, lh_forward, dt_, traj_com_duration_));
                        _rh_trajs.push_back(trajs::constant_traj(rh_init, dt_, traj_com_duration_));
                        lh_init = lh_forward;
                        break;
                    case States::MOVE_COM_LEFT:
                        com_lf(0) = lf_low.translation()(0);
                        _rf_trajs.push_back(trajs::constant_traj(rf_low, dt_, traj_com_duration_));
                        _lf_trajs.push_back(trajs::constant_traj(lf_low, dt_, traj_com_duration_));
                        _com_trajs.push_back(trajs::min_jerk_trajectory(com_rf, com_lf, dt_, traj_com_duration_));
                        _lh_trajs.push_back(trajs::constant_traj(lh_init, dt_, traj_com_duration_));
                        _rh_trajs.push_back(trajs::constant_traj(rh_init, dt_, traj_com_duration_));
                        break;
                    case States::LIFT_UP_RF:
                        rf_high.translation()(0) = lf_low.translation()(0);
                        _rf_trajs.push_back(trajs::min_jerk_trajectory(rf_low, rf_high, dt_, traj_foot_duration_));
                        _lf_trajs.push_back(trajs::constant_traj(lf_low, dt_, traj_foot_duration_));
                        _com_trajs.push_back(trajs::constant_traj(com_lf, dt_, traj_foot_duration_));
                        _lh_trajs.push_back(trajs::constant_traj(lh_init, dt_, traj_com_duration_));
                        _rh_trajs.push_back(trajs::constant_traj(rh_init, dt_, traj_com_duration_));
                        break;
                    case States::LIFT_DOWN_RF:
                        rf_low = translate(rf_low, 2 * step_length_, 0);
                        rh_forward = translate(rh_init, 2 * step_length_, 0);
                        _rf_trajs.push_back(trajs::min_jerk_trajectory(rf_high, rf_low, dt_, traj_foot_duration_));
                        _lf_trajs.push_back(trajs::constant_traj(lf_low, dt_, traj_foot_duration_));
                        _com_trajs.push_back(trajs::constant_traj(com_lf, dt_, traj_foot_duration_));
                        _lh_trajs.push_back(trajs::constant_traj(lh_init, dt_, traj_com_duration_));
                        _rh_trajs.push_back(trajs::min_jerk_trajectory(rh_init, rh_forward, dt_, traj_com_duration_));
                        rh_init = rh_forward;
                        break;
                    case States::MOVE_COM_RIGHT:
                        com_rf(0) = rf_low.translation()(0);
                        _rf_trajs.push_back(trajs::constant_traj(rf_low, dt_, traj_com_duration_));
                        _lf_trajs.push_back(trajs::constant_traj(lf_low, dt_, traj_com_duration_));
                        _com_trajs.push_back(trajs::min_jerk_trajectory(com_lf, com_rf, dt_, traj_com_duration_));
                        _lh_trajs.push_back(trajs::constant_traj(lh_init, dt_, traj_com_duration_));
                        _rh_trajs.push_back(trajs::constant_traj(rh_init, dt_, traj_com_duration_));
                        break;
                    case States::LIFT_UP_LF:
                        lf_high.translation()(0) = rf_low.translation()(0);
                        _rf_trajs.push_back(trajs::constant_traj(rf_low, dt_, traj_foot_duration_));
                        _lf_trajs.push_back(trajs::min_jerk_trajectory(lf_low, lf_high, dt_, traj_foot_duration_));
                        _com_trajs.push_back(trajs::constant_traj(com_rf, dt_, traj_foot_duration_));
                        _lh_trajs.push_back(trajs::constant_traj(lh_init, dt_, traj_com_duration_));
                        _rh_trajs.push_back(trajs::constant_traj(rh_init, dt_, traj_com_duration_));
                        break;
                    case States::LIFT_DOWN_LF_FINAL:
                        lf_low = translate(lf_low, step_length_, 0);
                        lh_forward = translate(lh_init, step_length_, 0);
                        _rf_trajs.push_back(trajs::constant_traj(rf_low, dt_, traj_foot_duration_));
                        _lf_trajs.push_back(trajs::min_jerk_trajectory(lf_high, lf_low, dt_, traj_foot_duration_));
                        _com_trajs.push_back(trajs::constant_traj(com_rf, dt_, traj_foot_duration_));
                        _lh_trajs.push_back(trajs::min_jerk_trajectory(lh_init, lh_forward, dt_, traj_com_duration_));
                        _rh_trajs.push_back(trajs::constant_traj(rh_init, dt_, traj_com_duration_));
                        break;
                    case States::MOVE_COM_CENTER_FINAL:
                        com_init.head(2) = (rf_low.translation().head(2) + lf_low.translation().head(2)) / 2.0;
                        _rf_trajs.push_back(trajs::constant_traj(rf_low, dt_, traj_com_duration_));
                        _lf_trajs.push_back(trajs::constant_traj(lf_low, dt_, traj_com_duration_));
                        _com_trajs.push_back(trajs::min_jerk_trajectory(com_rf, com_init, dt_, traj_com_duration_));
                        _lh_trajs.push_back(trajs::constant_traj(lh_forward, dt_, traj_com_duration_));
                        _rh_trajs.push_back(trajs::constant_traj(rh_forward, dt_, traj_com_duration_));
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

                auto controller = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_);

                if (run_) {

                    // add and remove contacts
                    if (time_ == 0 && (state_ == States::LIFT_UP_LF || state_ == States::LF_INIT))
                    {
                        controller->set_behavior_type(controllers::behavior_types::SINGLE_SUPPORT);
                        controller->remove_contact("contact_lfoot");
                    }
                    if (time_ == 0 && state_ == States::LIFT_UP_RF)
                    {
                        controller->set_behavior_type(controllers::behavior_types::SINGLE_SUPPORT);
                        controller->remove_contact("contact_rfoot");
                    }
                    if (time_ == _com_trajs[_current_traj].size() - 1 && (state_ == States::LIFT_DOWN_LF || state_ == LIFT_DOWN_LF_FINAL))
                    {
                        controller->set_behavior_type(controllers::behavior_types::DOUBLE_SUPPORT);
                        controller->add_contact("contact_lfoot");
                    }
                    if (time_ == _com_trajs[_current_traj].size() - 1 && state_ == States::LIFT_DOWN_RF)
                    {
                        controller->set_behavior_type(controllers::behavior_types::DOUBLE_SUPPORT);
                        controller->add_contact("contact_rfoot");
                    }

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