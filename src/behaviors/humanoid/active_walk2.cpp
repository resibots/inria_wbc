#include "inria_wbc/behaviors/humanoid/active_walk2.hpp"
#include "inria_wbc/estimators/cop.hpp"

namespace inria_wbc {
    namespace behaviors {
        namespace humanoid {
            static Register<ActiveWalk2> __ActiveWalk2("humanoid::active_walk2");

            ActiveWalk2::ActiveWalk2(const controller_ptr_t& controller, const YAML::Node& config) : Behavior(controller, config)
            {
                // check that the controller is compatible
                auto h_controller = std::dynamic_pointer_cast<inria_wbc::controllers::HumanoidPosTracker>(controller_);
                IWBC_ASSERT(h_controller != NULL, "ActiveWalk2: the controllers needs to be a HumanoidPosTracker (or related)!");
                IWBC_ASSERT(h_controller->has_task("lf_sole"), "active_walk: an lf_sole task is required (left foot)");
                IWBC_ASSERT(h_controller->has_task("rf_sole"), "active_walk: an rf_sole task is required (right foot)");
                IWBC_ASSERT(h_controller->has_task("lh"), "active_walk: an lh task is required (left hand)");
                IWBC_ASSERT(h_controller->has_task("rh"), "active_walk: an rh task is required (right hand)");
                IWBC_ASSERT(h_controller->has_task("com"), "active_walk: a com task is required");
                IWBC_ASSERT(h_controller->has_contact("contact_lfoot"), "active_walk: a contact_lfoot task is required");
                IWBC_ASSERT(h_controller->has_contact("contact_rfoot"), "active_walk: a contact_rfoot task is required");

                // load the parameters
                auto c = IWBC_CHECK(config["BEHAVIOR"]);
                transition_duration_ = IWBC_CHECK(c["transition_duration"].as<float>());
                traj_foot_duration_ = IWBC_CHECK(c["traj_foot_duration"].as<float>());

                if (transition_duration_ < 0.0)
                    IWBC_ERROR("transition_duration_ should >= 0.0");
                if (traj_foot_duration_ < 0.0)
                    IWBC_ERROR("traj_com_duration_ should >= 0.0");

                step_height_ = IWBC_CHECK(c["step_height"].as<float>());
                step_length_ = IWBC_CHECK(c["step_length"].as<float>());
                step_lateral_ = IWBC_CHECK(c["step_lateral"].as<float>());
                remove_contacts_ = IWBC_CHECK(c["remove_contacts"].as<bool>());

                if ((std::abs(step_height_) > 1e-5 || std::abs(step_length_) > 1e-5 || std::abs(step_lateral_) > 1e-5) && !remove_contacts_)
                    IWBC_ERROR("remove_contacts_ should be true to make a step");

                if (std::abs(step_height_) < 1e-5 && (std::abs(step_length_) > 1e-5 || std::abs(step_lateral_) > 1e-5))
                    IWBC_ERROR("step_height_ should be non zero true to make a step");

                force_treshold_ = IWBC_CHECK(c["force_treshold"].as<float>());
                num_cycles_ = IWBC_CHECK(c["num_cycles"].as<int>());

                com_percentage_ref_ = IWBC_CHECK(c["com_percentage_ref"].as<float>());
                if (com_percentage_ref_ >= 1.0 || com_percentage_ref_ <= 0.0)
                    IWBC_ERROR("com_percentage_ref_ should be between 0.0 and 1.0, it is ", com_percentage_ref_);

                com_percentage_foot_up_ = IWBC_CHECK(c["com_percentage_foot_up"].as<float>());
                if (com_percentage_foot_up_ >= 1.0 || com_percentage_foot_up_ <= 0.0)
                    IWBC_ERROR("com_percentage_foot_up_ should be between 0.0 and 1.0, it is ", com_percentage_foot_up_);
                if (com_percentage_foot_up_ >= com_percentage_ref_)
                    IWBC_ERROR("com_percentage_foot_up_ should < to com_percentage_ref_");

                send_vel_acc_ = IWBC_CHECK(c["send_vel_acc"].as<bool>());
                one_foot_ = IWBC_CHECK(c["one_foot"].as<bool>());

                if (one_foot_ && !remove_contacts_)
                    IWBC_ERROR("remove_contacts_ should be true to put Talos on one foot");

                error_cop_ = IWBC_CHECK(c["error_cop"].as<float>());
                if (error_cop_ < 0.0)
                    IWBC_ERROR("error_cop should be >= 0");

                behavior_type_ = this->behavior_type();
                controller_->set_behavior_type(behavior_type_);

                dt_ = controller_->dt();

                state_ = States::GO_TO_RF;

                com_init_ = h_controller->com();
                lh_init_ = h_controller->get_se3_ref("lh");
                rh_init_ = h_controller->get_se3_ref("rh");
                lf_init_ = h_controller->get_se3_ref("lf_sole");
                rf_init_ = h_controller->get_se3_ref("rf_sole");
                com_final_ = com_init_;
                com_foot_up_ = rh_init_.translation()(1);
                lh_final_ = lh_init_;
                rh_final_ = rh_init_;
                lf_final_ = lf_init_;
                rf_final_ = rf_init_;

                left_sole_name_ = h_controller->robot()->model().frames[h_controller->contact("contact_lfoot")->getMotionTask().frame_id()].name;
                right_sole_name_ = h_controller->robot()->model().frames[h_controller->contact("contact_rfoot")->getMotionTask().frame_id()].name;

                if (h_controller->robot()->model().frames[h_controller->task<tsid::tasks::TaskSE3Equality>("lf_sole")->frame_id()].name != left_sole_name_)
                    IWBC_ERROR("lf_sole and contact_lfoot should track the same frame");
                if (h_controller->robot()->model().frames[h_controller->task<tsid::tasks::TaskSE3Equality>("rf_sole")->frame_id()].name != right_sole_name_)
                    IWBC_ERROR("rf_sole and contact_rfoot should track the same frame");

                if (!h_controller->robot()->model().existFrame("v_left_sole_internal_border"))
                    IWBC_ERROR("There is no v_left_sole_internal_border virtual frame. Please add it in the dedicated virtual frames yaml");
                if (!h_controller->robot()->model().existFrame("v_left_sole_external_border"))
                    IWBC_ERROR("There is no v_left_sole_external_border virtual frame. Please add it in the dedicated virtual frames yaml");
                if (!h_controller->robot()->model().existFrame("v_right_sole_internal_border"))
                    IWBC_ERROR("There is no v_right_sole_internal_border virtual frame. Please add it in the dedicated virtual frames yaml");
                if (!h_controller->robot()->model().existFrame("v_right_sole_external_border"))
                    IWBC_ERROR("There is no v_right_sole_external_border virtual frame. Please add it in the dedicated virtual frames yaml");

                dt_ = h_controller->dt();
            }

            void ActiveWalk2::set_se3_ref(const pinocchio::SE3& init, const pinocchio::SE3& final, const std::string& task_name, const std::string& contact_name, const double& trajectory_duration, const int& index)
            {
                tsid::trajectories::TrajectorySample sample_ref(12, 6);
                Eigen::VectorXd ref_vec(12);
                auto ref = trajs::min_jerk_trajectory(init, final, dt_, trajectory_duration, index);
                tsid::math::SE3ToVector(ref, ref_vec);
                sample_ref.setValue(ref_vec);
                sample_ref.setDerivative(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(init, final, dt_, trajectory_duration, index));
                sample_ref.setSecondDerivative(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(init, final, dt_, trajectory_duration, index));
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

            Eigen::VectorXd ActiveWalk2::set_com_ref(const Eigen::VectorXd& init, const Eigen::VectorXd& final, const double& trajectory_duration, const int& index)
            {
                tsid::trajectories::TrajectorySample sample_ref(3, 3);
                auto ref = trajs::min_jerk_trajectory(init, final, dt_, trajectory_duration, index);
                sample_ref.setValue(ref);
                sample_ref.setDerivative(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(init, final, dt_, trajectory_duration, index));
                sample_ref.setSecondDerivative(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(init, final, dt_, trajectory_duration, index));
                if (send_vel_acc_)
                    std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_com_ref(sample_ref);
                else
                    std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_com_ref(ref);
                return ref;
            }

            void ActiveWalk2::update(const controllers::SensorData& sensor_data)
            {
                if (sensor_data.find("lf_force") == sensor_data.end())
                    IWBC_ERROR("ActiveWalk2 needs the LF force");

                if (sensor_data.find("rf_force") == sensor_data.end())
                    IWBC_ERROR("ActiveWalk2 needs the LF force");

                auto controller = std::dynamic_pointer_cast<inria_wbc::controllers::HumanoidPosTracker>(controller_);
                Eigen::Vector2d cop = Eigen::Vector2d::Zero();
                if (controller->cop())
                    cop = controller->cop().value();
                bool keep_sending_com_traj = true;

                //GO_TO_RF: PUT COM ON RF
                if (state_ == States::GO_TO_RF) {
                    if (begin_) {
                        com_init_ = controller->com();
                        com_final_ = com_init_;
                        com_final_(0) = controller->model_frame_pos(right_sole_name_).translation()(0);

                        auto internal_rf_border = controller->model_frame_pos("v_right_sole_internal_border").translation()[1];
                        auto external_rf_border = controller->model_frame_pos("v_right_sole_external_border").translation()[1];
                        com_final_(1) = internal_rf_border + com_percentage_ref_ * (external_rf_border - internal_rf_border);
                        com_foot_up_ = internal_rf_border + com_percentage_foot_up_ * (external_rf_border - internal_rf_border);

                        lf_init_ = controller->model_frame_pos(left_sole_name_);
                        lf_final_ = lf_init_;
                        lf_final_.translation()(2) += step_height_;

                        begin_ = false;
                        index_ = 0;
                        time_index_ = 0;
                        remove_contact_index_ = 0;
                    }

                    if (controller->cop()) {
                        keep_sending_com_traj = (cop - com_final_.head(2)).norm() > error_cop_;
                        std::cout << (cop - com_final_.head(2)).norm() << std::endl;
                        // std::cout << cop(1) << " "  << com_foot_up_ << std::endl;
                    }

                    if (index_ < std::floor(traj_foot_duration_ / dt_)) {
                        bool lift_foot_up = false;
                        if (keep_sending_com_traj) {
                            auto ref = set_com_ref(com_init_, com_final_, traj_foot_duration_, index_);
                            lift_foot_up = ref(1) < com_foot_up_;
                        }
                        else {
                            lift_foot_up = true;
                        }
                        if (lift_foot_up) {
                            if (remove_contacts_ && controller->activated_contacts_forces().find("contact_lfoot") != controller->activated_contacts_forces().end()) {
                                controller->remove_contact("contact_lfoot");
                                remove_contact_index_ = index_;
                            }
                            if (time_index_ < std::floor(traj_foot_duration_ / dt_ - remove_contact_index_)) {
                                set_se3_ref(lf_init_, lf_final_, "lf_sole", "contact_lfoot", traj_foot_duration_ - remove_contact_index_ * dt_, time_index_);
                                time_index_++;
                            }
                        }
                        index_++;
                    }
                    else {
                        state_ = States::LIFT_DOWN_LF;
                        begin_ = true;
                    }
                }

                //LIFT_DOWN_LF: PUT LEFT FOOT ON THE GROUND
                if (state_ == States::LIFT_DOWN_LF) {

                    if (begin_) {
                        lf_init_ = controller->model_frame_pos(left_sole_name_);
                        lf_final_ = lf_init_;
                        if (first_step_ && cycle_count_ != num_cycles_)
                            lf_final_.translation()(1) += step_lateral_;
                        lf_final_.translation()(2) = 0.0;
                        lf_final_.translation()(0) += first_step_ ? step_length_ : 2 * step_length_;
                        lh_init_ = controller->get_se3_ref("lh");
                        lh_final_ = lh_init_;
                        lh_final_.translation()(0) = lf_final_.translation()(0) + 0.3;
                        lh_final_.translation()(1) = lf_final_.translation()(1) + 0.3;
                        begin_ = false;
                        index_ = 0;
                    }

                    if (index_ < std::floor(transition_duration_ / dt_) && sensor_data.at("lf_force")(2) < force_treshold_) {
                        set_se3_ref(lf_init_, lf_final_, "lf_sole", "contact_lfoot", transition_duration_, index_);
                        set_se3_ref(lh_init_, lh_final_, "lh", "", transition_duration_, index_);
                        index_++;
                    }
                    else {
                        if (remove_contacts_)
                            controller->add_contact("contact_lfoot");

                        if (cycle_count_ == num_cycles_) {
                            next_state_ = States::GO_TO_MIDDLE;
                            state_ = States::GO_TO_MIDDLE;
                        }
                        else {
                            state_ = States::GO_TO_MIDDLE;
                            next_state_ = States::GO_TO_LF;
                        }
                        begin_ = true;
                    }
                }

                //GO_TO_MIDDLE: PUT COM BETWEEN THE TWO FEET
                if (state_ == States::GO_TO_MIDDLE) {
                    if (begin_) {
                        com_init_ = controller->com();
                        com_final_ = com_init_;
                        com_final_.head(2) = 0.5 * (controller->model_frame_pos(left_sole_name_).translation().head(2) + controller->model_frame_pos(right_sole_name_).translation().head(2));
                        begin_ = false;
                        index_ = 0;
                    }

                    if (controller->cop())
                        keep_sending_com_traj = (cop - com_final_.head(2)).norm() > error_cop_;

                    if (index_ < std::floor(transition_duration_ / dt_) && keep_sending_com_traj) {
                        set_com_ref(com_init_, com_final_, transition_duration_, index_);
                        index_++;
                    }
                    else {
                        state_ = next_state_;
                        begin_ = true;
                    }
                }

                if (state_ == States::GO_TO_LF) {
                    if (begin_) {
                        com_init_ = controller->com();
                        com_final_ = com_init_;
                        com_final_(0) = controller->model_frame_pos(left_sole_name_).translation()(0);

                        auto internal_lf_border = controller->model_frame_pos("v_left_sole_internal_border").translation()[1];
                        auto external_lf_border = controller->model_frame_pos("v_left_sole_external_border").translation()[1];
                        com_final_(1) = internal_lf_border + com_percentage_ref_ * (external_lf_border - internal_lf_border);
                        com_foot_up_ = internal_lf_border + com_percentage_foot_up_ * (external_lf_border - internal_lf_border);

                        rf_init_ = controller->model_frame_pos(right_sole_name_);
                        rf_final_ = rf_init_;
                        rf_final_.translation()(2) += step_height_;

                        begin_ = false;
                        index_ = 0;
                        time_index_ = 0;
                        remove_contact_index_ = 0;
                    }

                    if (controller->cop()) {
                        keep_sending_com_traj = (cop - com_final_.head(2)).norm() > error_cop_;
                    }

                    if (index_ < std::floor(traj_foot_duration_ / dt_)) {
                        bool lift_foot_up = false;
                        if (keep_sending_com_traj) {
                            auto ref = set_com_ref(com_init_, com_final_, traj_foot_duration_, index_);
                            lift_foot_up = ref(1) < com_foot_up_;
                        }
                        else {
                            lift_foot_up = true;
                        }
                        if (lift_foot_up) {
                            if (remove_contacts_ && controller->activated_contacts_forces().find("contact_rfoot") != controller->activated_contacts_forces().end()) {
                                controller->remove_contact("contact_rfoot");
                                remove_contact_index_ = index_;
                            }
                            if (time_index_ < std::floor(traj_foot_duration_ / dt_ - remove_contact_index_)) {
                                set_se3_ref(rf_init_, rf_final_, "rf_sole", "contact_rfoot", traj_foot_duration_ - remove_contact_index_ * dt_, time_index_);
                                time_index_++;
                            }
                        }
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
                        rf_init_ = controller->model_frame_pos(right_sole_name_);
                        rf_final_ = rf_init_;
                        if (first_step_ && cycle_count_ != num_cycles_) {
                            rf_final_.translation()(1) -= step_lateral_;
                            first_step_ = false;
                        }
                        rf_final_.translation()(2) = 0.0;
                        rf_final_.translation()(0) += first_step_ ? step_length_ : 2 * step_length_;
                        rh_init_ = controller->get_se3_ref("rh");
                        rh_final_ = rh_init_;
                        rh_final_.translation()(0) = rf_final_.translation()(0) + 0.3;
                        rh_final_.translation()(1) = rf_final_.translation()(1) - 0.3;
                        begin_ = false;
                        index_ = 0;
                        if (first_step_)
                            first_step_ = false;
                    }
                    if (index_ < std::floor(transition_duration_ / dt_) && sensor_data.at("rf_force")(2) < force_treshold_) {
                        set_se3_ref(rf_init_, rf_final_, "rf_sole", "contact_rfoot", transition_duration_, index_);
                        set_se3_ref(rh_init_, rh_final_, "rh", "", transition_duration_, index_);
                        index_++;
                    }
                    else {
                        if (remove_contacts_)
                            controller->add_contact("contact_rfoot");

                        begin_ = true;
                        cycle_count_++;

                        if (cycle_count_ == num_cycles_)
                            first_step_ = true;

                        state_ = States::GO_TO_MIDDLE;
                        next_state_ = States::GO_TO_RF;
                    }
                }

                controller->update(sensor_data);
            }
        } // namespace humanoid
    } // namespace behaviors
} // namespace inria_wbc
