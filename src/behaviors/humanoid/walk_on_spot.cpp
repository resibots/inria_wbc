#include "inria_wbc/behaviors/humanoid/walk_on_spot.hpp"
#include <inria_wbc/trajs/utils.hpp>

namespace inria_wbc {
    namespace behaviors {
        namespace humanoid {
            static Register<WalkOnSpot> __walk_on_spot("humanoid::walk-on-spot");

            WalkOnSpot::WalkOnSpot(const controller_ptr_t& controller, const YAML::Node& config) : Behavior(controller, config)
            {
                // check that the controller is compatible
                auto h_controller = std::dynamic_pointer_cast<inria_wbc::controllers::HumanoidPosTracker>(controller_);
                IWBC_ASSERT(h_controller != NULL, "Walk on spot: the controllers needs to be a HumanoidPosTracker (or related)!");
                IWBC_ASSERT(h_controller->has_task("lf"), "Walk on spot: an lf task is required (left foot)");
                IWBC_ASSERT(h_controller->has_task("rf"), "Walk on spot: an rf task is required (right foot)");
                IWBC_ASSERT(h_controller->has_task("com"), "Walk: a com task is required");
                IWBC_ASSERT(h_controller->has_contact("contact_lfoot"), "Walk on spot: a contact_lfoot task is required");
                IWBC_ASSERT(h_controller->has_contact("contact_rfoot"), "Walk on spot: a contact_rfoot task is required");

                // load parameters
                auto c = IWBC_CHECK(config["BEHAVIOR"]);
                traj_com_duration_ = IWBC_CHECK(c["traj_com_duration"].as<float>());
                traj_foot_duration_ = IWBC_CHECK(c["traj_foot_duration"].as<float>());
                step_height_ = IWBC_CHECK(c["step_height"].as<float>());

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

                auto controller = std::dynamic_pointer_cast<inria_wbc::controllers::HumanoidPosTracker>(controller_);
                IWBC_ASSERT(controller, "Walk on spot requires a humanoid controller!");
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
                        _rf_trajs.push_back(trajs::to_sample_trajectory(trajs::constant_traj(rf_low, dt_, traj_com_duration_)));
                        _lf_trajs.push_back(trajs::to_sample_trajectory(trajs::constant_traj(lf_low, dt_, traj_com_duration_)));
                        _com_trajs.push_back(
                            trajs::to_sample_trajectory(
                                trajs::min_jerk_trajectory(com_init, com_rf, dt_, traj_com_duration_)/*,
                                trajs::min_jerk_trajectory<trajs::d_order::FIRST>(com_init, com_rf, dt_, traj_com_duration_),
                                trajs::min_jerk_trajectory<trajs::d_order::SECOND>(com_init, com_rf, dt_, traj_com_duration_)
                            */)
                        );
                        break;
                    case States::LIFT_UP_LF:
                        _rf_trajs.push_back(trajs::to_sample_trajectory(trajs::constant_traj(rf_low, dt_, traj_foot_duration_)));
                        _lf_trajs.push_back(
                            trajs::to_sample_trajectory(
                                trajs::min_jerk_trajectory(lf_low, lf_high, dt_, traj_foot_duration_)/*,
                                trajs::min_jerk_trajectory<trajs::d_order::FIRST>(lf_low, lf_high, dt_, traj_foot_duration_),
                                trajs::min_jerk_trajectory<trajs::d_order::SECOND>(lf_low, lf_high, dt_, traj_foot_duration_)
                            */)
                        );
                        _com_trajs.push_back(trajs::to_sample_trajectory(trajs::constant_traj(com_rf, dt_, traj_foot_duration_)));
                        break;
                    case States::LIFT_DOWN_LF:
                        _rf_trajs.push_back(trajs::to_sample_trajectory(trajs::constant_traj(rf_low, dt_, traj_foot_duration_)));
                        _lf_trajs.push_back(
                            trajs::to_sample_trajectory(
                                trajs::min_jerk_trajectory(lf_high, lf_low, dt_, traj_foot_duration_)/*,
                                trajs::min_jerk_trajectory<trajs::d_order::FIRST>(lf_high, lf_low, dt_, traj_foot_duration_),
                                trajs::min_jerk_trajectory<trajs::d_order::SECOND>(lf_high, lf_low, dt_, traj_foot_duration_)
                            */)
                        );
                        _com_trajs.push_back(trajs::to_sample_trajectory(trajs::constant_traj(com_rf, dt_, traj_foot_duration_)));
                        break;
                    case States::MOVE_COM_LEFT:
                        _rf_trajs.push_back(trajs::to_sample_trajectory(trajs::constant_traj(rf_low, dt_, traj_com_duration_)));
                        _lf_trajs.push_back(trajs::to_sample_trajectory(trajs::constant_traj(lf_low, dt_, traj_com_duration_)));
                        _com_trajs.push_back(
                            trajs::to_sample_trajectory(
                                trajs::min_jerk_trajectory(com_rf, com_lf, dt_, traj_com_duration_)/*,
                                trajs::min_jerk_trajectory<trajs::d_order::FIRST>(com_rf, com_lf, dt_, traj_com_duration_),
                                trajs::min_jerk_trajectory<trajs::d_order::SECOND>(com_rf, com_lf, dt_, traj_com_duration_)
                            */)
                        );
                        break;
                    case States::LIFT_UP_RF:
                        _rf_trajs.push_back(
                            trajs::to_sample_trajectory(
                                trajs::min_jerk_trajectory(rf_low, rf_high, dt_, traj_foot_duration_)/*,
                                trajs::min_jerk_trajectory<trajs::d_order::FIRST>(rf_low, rf_high, dt_, traj_foot_duration_),
                                trajs::min_jerk_trajectory<trajs::d_order::SECOND>(rf_low, rf_high, dt_, traj_foot_duration_)
                            */)
                        );
                        _lf_trajs.push_back(trajs::to_sample_trajectory(trajs::constant_traj(lf_low, dt_, traj_foot_duration_)));
                        _com_trajs.push_back(trajs::to_sample_trajectory(trajs::constant_traj(com_lf, dt_, traj_foot_duration_)));
                        break;
                    case States::LIFT_DOWN_RF:
                        _rf_trajs.push_back(
                            trajs::to_sample_trajectory(
                                trajs::min_jerk_trajectory(rf_high, rf_low, dt_, traj_foot_duration_)/*,
                                trajs::min_jerk_trajectory<trajs::d_order::FIRST>(rf_high, rf_low, dt_, traj_foot_duration_),
                                trajs::min_jerk_trajectory<trajs::d_order::SECOND>(rf_high, rf_low, dt_, traj_foot_duration_)
                            */)
                        );
                        _lf_trajs.push_back(trajs::to_sample_trajectory(trajs::constant_traj(lf_low, dt_, traj_foot_duration_)));
                        _com_trajs.push_back(trajs::to_sample_trajectory(trajs::constant_traj(com_lf, dt_, traj_foot_duration_)));
                        break;
                    case States::MOVE_COM_RIGHT:
                        _rf_trajs.push_back(trajs::to_sample_trajectory(trajs::constant_traj(rf_low, dt_, traj_com_duration_)));
                        _lf_trajs.push_back(trajs::to_sample_trajectory(trajs::constant_traj(lf_low, dt_, traj_com_duration_)));
                        _com_trajs.push_back(
                            trajs::to_sample_trajectory(
                                trajs::min_jerk_trajectory(com_lf, com_rf, dt_, traj_com_duration_)/*,
                                trajs::min_jerk_trajectory<trajs::d_order::FIRST>(com_lf, com_rf, dt_, traj_com_duration_),
                                trajs::min_jerk_trajectory<trajs::d_order::SECOND>(com_lf, com_rf, dt_, traj_com_duration_)
                            */)
                        );
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
                auto controller = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_);

                // add and remove contacts
                if (time_ == 0 && state_ == States::LIFT_UP_LF)
                {
                    controller->set_behavior_type(controllers::behavior_types::SINGLE_SUPPORT);
                    controller->remove_contact("contact_lfoot");
                }
                if (time_ == 0 && state_ == States::LIFT_UP_RF)
                {
                    controller->set_behavior_type(controllers::behavior_types::SINGLE_SUPPORT);
                    controller->remove_contact("contact_rfoot");
                }
                if (time_ == _com_trajs[_current_traj].size() - 1 && state_ == States::LIFT_DOWN_LF)
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

                pinocchio::SE3 lf_se3 = trajs::se3_from_sample(_lf_trajs[_current_traj][time_]);
                pinocchio::SE3 rf_se3 = trajs::se3_from_sample(_rf_trajs[_current_traj][time_]);

                controller->set_com_ref(_com_trajs[_current_traj][time_]);
                controller->set_se3_ref(_lf_trajs[_current_traj][time_], "lf");
                controller->set_se3_ref(_rf_trajs[_current_traj][time_], "rf");
                controller->set_contact_se3_ref(lf_se3, "contact_lfoot");
                controller->set_contact_se3_ref(rf_se3, "contact_rfoot");

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