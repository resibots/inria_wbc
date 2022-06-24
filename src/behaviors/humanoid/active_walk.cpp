#include "inria_wbc/behaviors/humanoid/active_walk.hpp"
#include "inria_wbc/estimators/cop.hpp"

namespace inria_wbc {
    namespace behaviors {
        namespace humanoid {
            static Register<ActiveWalk> __ActiveWalk("humanoid::active_walk");

            ActiveWalk::ActiveWalk(const controller_ptr_t& controller, const YAML::Node& config) : Behavior(controller, config)
            {
                // check that the controller is compatible
                auto h_controller = std::dynamic_pointer_cast<inria_wbc::controllers::HumanoidPosTracker>(controller_);
                IWBC_ASSERT(h_controller != NULL, "ActiveWalk: the controllers needs to be a HumanoidPosTracker (or related)!");
                IWBC_ASSERT(h_controller->has_task("lf"), "active_walk: an lf task is required (left foot)");
                IWBC_ASSERT(h_controller->has_task("rf"), "active_walk: an rf task is required (right foot)");
                IWBC_ASSERT(h_controller->has_task("lh"), "active_walk: an lh task is required (left hand)");
                IWBC_ASSERT(h_controller->has_task("rh"), "active_walk: an rh task is required (right hand)");
                IWBC_ASSERT(h_controller->has_task("com"), "active_walk: a com task is required");
                IWBC_ASSERT(h_controller->has_contact("contact_lfoot"), "active_walk: a contact_lfoot task is required");
                IWBC_ASSERT(h_controller->has_contact("contact_rfoot"), "active_walk: a contact_rfoot task is required");

                // load the parameters
                auto c = IWBC_CHECK(config["BEHAVIOR"]);
                traj_com_duration_ = IWBC_CHECK(c["traj_com_duration"].as<float>());
                traj_foot_duration_ = IWBC_CHECK(c["traj_foot_duration"].as<float>());
                traj_stop_duration_ = IWBC_CHECK(c["traj_stop_duration"].as<float>());

                if (traj_com_duration_ < 0.0)
                    IWBC_ERROR("traj_com_duration_ should >= 0.0");
                if (traj_foot_duration_ < 0.0)
                    IWBC_ERROR("traj_com_duration_ should >= 0.0");
                if (traj_stop_duration_ < 0.0)
                    IWBC_ERROR("traj_com_duration_ should >= 0.0");

                step_height_ = IWBC_CHECK(c["step_height"].as<float>());
                step_length_ = IWBC_CHECK(c["step_length"].as<float>());
                step_lateral_ = IWBC_CHECK(c["step_lateral"].as<float>());
                remove_contacts_ = IWBC_CHECK(c["remove_contacts"].as<bool>());

                if ((std::abs(step_height_) < 1e-5 || std::abs(step_length_) < 1e-5) && !remove_contacts_)
                    IWBC_ERROR("remove_contacts_ should be true to make a step");

                force_treshold_ = IWBC_CHECK(c["force_treshold"].as<float>());
                num_cycles_ = IWBC_CHECK(c["num_cycles"].as<int>());

                com_percentage_ = IWBC_CHECK(c["com_percentage"].as<float>());
                if (com_percentage_ >= 1.0 || com_percentage_ <= 0.0)
                    IWBC_ERROR("com_percentage_ should be between 0.0 and 1.0, it is ", com_percentage_);

                go_to_middle_ = IWBC_CHECK(c["go_to_middle"].as<bool>());
                send_vel_acc_ = IWBC_CHECK(c["send_vel_acc"].as<bool>());
                one_foot_ = IWBC_CHECK(c["one_foot"].as<bool>());

                if (one_foot_ && !remove_contacts_)
                    IWBC_ERROR("remove_contacts_ should be true to put Talos on one foot");

                behavior_type_ = this->behavior_type();
                controller_->set_behavior_type(behavior_type_);

                dt_ = controller_->dt();
                time_ = 0;

                state_ = States::GO_TO_RF;

                com_init_ = h_controller->com();
                lh_init_ = h_controller->get_se3_ref("lh");
                rh_init_ = h_controller->get_se3_ref("rh");
                lf_init_ = h_controller->get_se3_ref("lf");
                rf_init_ = h_controller->get_se3_ref("rf");
                lh_init_ = h_controller->get_se3_ref("lh");
                rh_init_ = h_controller->get_se3_ref("rh");
                com_final_ = com_init_;
                lh_final_ = lh_init_;
                rh_final_ = rh_init_;
                lf_final_ = lf_init_;
                rf_final_ = rf_init_;
                lh_final_ = lh_init_;
                rh_final_ = rh_init_;

                left_ankle_name_ = h_controller->robot()->model().frames[h_controller->contact("contact_lfoot")->getMotionTask().frame_id()].name;
                right_ankle_name_ = h_controller->robot()->model().frames[h_controller->contact("contact_rfoot")->getMotionTask().frame_id()].name;
                left_sole_lyn_ = std::abs(h_controller->contact("contact_lfoot")->getContactPoints()(1, 0));
                left_sole_lyp_ = std::abs(h_controller->contact("contact_lfoot")->getContactPoints()(1, 1));
                right_sole_lyn_ = std::abs(h_controller->contact("contact_rfoot")->getContactPoints()(1, 0));
                right_sole_lyp_ = std::abs(h_controller->contact("contact_rfoot")->getContactPoints()(1, 1));

                dt_ = h_controller->dt();
            }

            void ActiveWalk::set_se3_ref(const pinocchio::SE3& init, const pinocchio::SE3& final, const std::string& task_name, const std::string& contact_name)
            {
                tsid::trajectories::TrajectorySample sample_ref(12, 6);
                Eigen::VectorXd ref_vec(12);
                auto ref = trajs::min_jerk_trajectory(init, final, dt_, traj_foot_duration_, index_);
                tsid::math::SE3ToVector(ref, ref_vec);
                sample_ref.setValue(ref_vec);
                sample_ref.setDerivative(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(init, final, dt_, traj_foot_duration_, index_));
                sample_ref.setSecondDerivative(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(init, final, dt_, traj_foot_duration_, index_));
                if (send_vel_acc_) {
                    std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_se3_ref(ref, task_name);
                    if (contact_name != "")
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_contact_se3_ref(ref, contact_name);
                }
                else {
                    std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_se3_ref(sample_ref, task_name);
                    if (contact_name != "")
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_contact_se3_ref(sample_ref, contact_name);
                }
            }

            void ActiveWalk::set_com_ref(const Eigen::VectorXd& init, const Eigen::VectorXd& final)
            {
                tsid::trajectories::TrajectorySample sample_ref(3, 3);
                auto ref = trajs::min_jerk_trajectory(init, final, dt_, traj_com_duration_, index_);
                sample_ref.setValue(ref);
                sample_ref.setDerivative(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(init, final, dt_, traj_com_duration_, index_));
                sample_ref.setSecondDerivative(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(init, final, dt_, traj_com_duration_, index_));
                if (send_vel_acc_)
                    std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_com_ref(sample_ref);
                else
                    std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_com_ref(ref);
            }

            void ActiveWalk::update(const controllers::SensorData& sensor_data)
            {
                if (sensor_data.find("lf_force") == sensor_data.end())
                    IWBC_ERROR("ActiveWalk needs the LF force");

                if (sensor_data.find("rf_force") == sensor_data.end())
                    IWBC_ERROR("ActiveWalk needs the LF force");

                auto controller = std::dynamic_pointer_cast<inria_wbc::controllers::HumanoidPosTracker>(controller_);

                //GO_TO_MIDDLE: PUT COM BETWEEN THE TWO FEET
                if (state_ == States::GO_TO_MIDDLE) {
                    if (begin_) {
                        com_init_ = controller->com();
                        com_final_ = com_init_;
                        com_final_.head(2) = 0.5 * (controller->get_se3_ref("rf").translation().head(2) + controller->get_se3_ref("lf").translation().head(2));
                        begin_ = false;
                        index_ = 0;
                    }

                    if (index_ < std::floor(traj_com_duration_ / dt_)) {
                        set_com_ref(com_init_, com_final_);
                        index_++;
                    }
                    else {
                        state_ = States::STOP;
                        begin_ = true;
                    }
                }

                if (state_ == States::STOP) {
                    if (begin_) {
                        index_ = 0;
                        begin_ = false;
                    }
                    if (index_ < std::floor(traj_stop_duration_ / dt_))
                        index_++;
                    else {
                        state_ = next_state_;
                        begin_ = true;
                    }
                }
                //GO_TO_RF: PUT COM ON RF
                if (state_ == States::GO_TO_RF) {
                    if (begin_) {
                        com_init_ = controller->com();
                        com_final_ = com_init_;
                        com_final_(0) = controller->get_se3_ref("rf").translation()(0);
                        auto y_ankle = controller->get_se3_ref("rf").translation()(1);
                        auto internal_rf_border = y_ankle - sign(y_ankle) * right_sole_lyp_;
                        auto external_rf_border = y_ankle + sign(y_ankle) * right_sole_lyn_;
                        com_final_(1) = internal_rf_border + com_percentage_ * (external_rf_border - internal_rf_border);
                        begin_ = false;
                        index_ = 0;
                    }

                    if (index_ < std::floor(traj_com_duration_ / dt_)) {
                        set_com_ref(com_init_, com_final_);
                        index_++;
                    }
                    else {
                        state_ = States::LIFT_UP_LF;
                        begin_ = true;
                    }
                }

                //LIFT_UP_LF: LIFT LEFT FOOT OFF THE GROUND
                if (state_ == States::LIFT_UP_LF) {
                    if (begin_) {
                        if (remove_contacts_)
                            controller->remove_contact("contact_lfoot");
                        lf_init_ = controller->model_frame_pos(left_ankle_name_);
                        lf_final_ = lf_init_;
                        lf_final_.translation()(2) += step_height_;
                        begin_ = false;
                        index_ = 0;
                    }

                    if (index_ < std::floor(traj_foot_duration_ / dt_)) {
                        set_se3_ref(lf_init_, lf_final_, "lf", "contact_lfoot");
                        index_++;
                    }
                    else {
                        if (one_foot_) {
                            state_ = States::STOP;
                            next_state_ = States::STOP;
                        }
                        else {
                            state_ = States::LIFT_DOWN_LF;
                            begin_ = true;
                        }
                    }
                }

                //LIFT_DOWN_LF: PUT LEFT FOOT ON THE GROUND
                if (state_ == States::LIFT_DOWN_LF) {

                    if (begin_) {
                        lf_init_ = controller->model_frame_pos(left_ankle_name_);
                        lf_final_ = lf_init_;
                        if (first_step_ && cycle_count_ != num_cycles_)
                            lf_final_.translation()(1) += step_lateral_;
                        lf_final_.translation()(2) -= step_height_;
                        lf_final_.translation()(0) += first_step_ ? step_length_ : 2 * step_length_;
                        lh_init_ = controller->robot()->framePosition(controller->tsid()->data(), controller->task<tsid::tasks::TaskSE3Equality>("lh")->frame_id());
                        lh_final_ = lh_init_;
                        lh_final_.translation()(0) += first_step_ ? step_length_ : 2 * step_length_;
                        begin_ = false;
                        index_ = 0;
                    }

                    if (index_ < std::floor(traj_foot_duration_ / dt_) && sensor_data.at("lf_force")(2) < force_treshold_) {
                        set_se3_ref(lf_init_, lf_final_, "lf", "contact_lfoot");
                        set_se3_ref(lh_init_, lh_final_, "lh", "");
                        index_++;
                    }
                    else {
                        if (remove_contacts_)
                            controller->add_contact("contact_lfoot");

                        if (cycle_count_ == num_cycles_) {
                            next_state_ = States::GO_TO_MIDDLE;
                            state_ = States::GO_TO_MIDDLE;
                        }
                        else if (go_to_middle_) {
                            state_ = States::GO_TO_MIDDLE;
                            next_state_ = States::GO_TO_LF;
                        }
                        else {
                            state_ = States::GO_TO_LF;
                        }
                        begin_ = true;
                    }
                }

                //GO_TO_LF: PUT COM ON LEFT FOOT
                if (state_ == States::GO_TO_LF) {
                    if (begin_) {
                        com_init_ = controller->com();
                        com_final_ = com_init_;
                        com_final_(0) = controller->get_se3_ref("lf").translation()(0);
                        auto y_ankle = controller->get_se3_ref("lf").translation()(1);
                        auto internal_lf_border = y_ankle - sign(y_ankle) * left_sole_lyp_;
                        auto external_lf_border = y_ankle + sign(y_ankle) * left_sole_lyn_;
                        com_final_(1) = internal_lf_border + com_percentage_ * (external_lf_border - internal_lf_border);
                        begin_ = false;
                        index_ = 0;
                    }

                    if (index_ < std::floor(traj_com_duration_ / dt_)) {
                        set_com_ref(com_init_, com_final_);
                        index_++;
                    }
                    else {
                        state_ = States::LIFT_UP_RF;
                        begin_ = true;
                    }
                }

                //LIFT_UP_RF: LIFT RIGHT FOOT OFF THE GROUND
                if (state_ == States::LIFT_UP_RF) {
                    if (begin_) {
                        if (remove_contacts_)
                            controller->remove_contact("contact_rfoot");
                        rf_init_ = controller->model_frame_pos(right_ankle_name_);
                        rf_final_ = rf_init_;
                        rf_final_.translation()(2) += step_height_;
                        begin_ = false;
                        index_ = 0;
                    }

                    if (index_ < std::floor(traj_foot_duration_ / dt_)) {
                        set_se3_ref(rf_init_, rf_final_, "rf", "contact_rfoot");
                        index_++;
                    }
                    else {
                        state_ = States::LIFT_DOWN_RF;
                        begin_ = true;
                    }
                }

                //LIFT_DOWN_RF: PUT RIGHT FOOT ON THE GROUND
                if (state_ == States::LIFT_DOWN_RF) {

                    if (begin_) {
                        rf_init_ = controller->model_frame_pos(right_ankle_name_);
                        rf_final_ = rf_init_;
                        if (first_step_ && cycle_count_ != num_cycles_) {
                            rf_final_.translation()(1) -= step_lateral_;
                            first_step_ = false;
                        }
                        rf_final_.translation()(2) -= step_height_;
                        rf_final_.translation()(0) += first_step_ ? step_length_ : 2 * step_length_;
                        rh_init_ = controller->robot()->framePosition(controller->tsid()->data(), controller->task<tsid::tasks::TaskSE3Equality>("rh")->frame_id());
                        rh_final_ = rh_init_;
                        rh_final_.translation()(0) += first_step_ ? step_length_ : 2 * step_length_;
                        begin_ = false;
                        index_ = 0;
                        if (first_step_)
                            first_step_ = false;
                    }
                    if (index_ < std::floor(traj_foot_duration_ / dt_) && sensor_data.at("rf_force")(2) < force_treshold_) {
                        set_se3_ref(rf_init_, rf_final_, "rf", "contact_rfoot");
                        set_se3_ref(rh_init_, rh_final_, "rh", "");
                        index_++;
                    }
                    else {
                        if (remove_contacts_)
                            controller->add_contact("contact_rfoot");

                        begin_ = true;
                        cycle_count_++;

                        if (go_to_middle_) {
                            state_ = States::GO_TO_MIDDLE;
                            next_state_ = States::GO_TO_RF;
                        }
                        else {
                            state_ = States::GO_TO_RF;
                        }

                        if (cycle_count_ == num_cycles_)
                            first_step_ = true;
                    }
                }
                controller->update(sensor_data);
            }

        } // namespace humanoid
    } // namespace behaviors
} // namespace inria_wbc
