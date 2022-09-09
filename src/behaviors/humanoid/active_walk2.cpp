#include "inria_wbc/behaviors/humanoid/active_walk2.hpp"
#include "inria_wbc/estimators/cop.hpp"
#include "inria_wbc/stabilizers/stabilizer.hpp"

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
                IWBC_ASSERT(h_controller->has_task("torso"), "active_walk: a torso task is required");
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
                std::cout << "error_cop_ " << error_cop_ << std::endl;

                activate_error_cop_ = IWBC_CHECK(c["activate_error_cop"].as<bool>());

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

                torso_link_name_ = h_controller->robot()->model().frames[h_controller->task<tsid::tasks::TaskSE3Equality>("torso")->frame_id()].name;
                lh_delta_init_ = h_controller->get_se3_ref("lh").translation() - controller->model_frame_pos(torso_link_name_).translation();
                rh_delta_init_ = h_controller->get_se3_ref("rh").translation() - controller->model_frame_pos(torso_link_name_).translation();

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

                error_posture_ = IWBC_CHECK(c["error_posture"].as<float>());
                if (error_posture_ < 0.0)
                    IWBC_ERROR("Error_posture_ should be >= 0.0");
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
                    std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_se3_ref(ref, task_name);
                    if (contact_name != "")
                        std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_contact_se3_ref(ref, contact_name);
                }
                else {
                    std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_se3_ref(sample_ref, task_name);
                    if (contact_name != "")
                        std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_contact_se3_ref(sample_ref, contact_name);
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
                    std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_com_ref(sample_ref);
                else
                    std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_com_ref(ref);
                return ref;
            }

            // cop = (left_cop*left_force + right_cop*right_force)*(1/(left_force+right_force))
            // here we have global cop from reference, we can fix the foot cop (other_foot_cop) that will stay on the ground to the middle of the foot.
            // result is the opposite foot cop that will be lifted
            Eigen::Vector2d compute_foot_cop(const Eigen::Vector2d& other_foot_cop, const Eigen::Vector2d& total_cop, double other_foot_f, double f)
            {
                return (total_cop * (other_foot_f + f) - other_foot_cop * other_foot_f) * (1 / f);
            }

            void ActiveWalk2::update(const controllers::SensorData& sensor_data)
            {
                if (sensor_data.find("lf_force") == sensor_data.end())
                    IWBC_ERROR("ActiveWalk2 needs the LF force");

                if (sensor_data.find("rf_force") == sensor_data.end())
                    IWBC_ERROR("ActiveWalk2 needs the LF force");

                if (sensor_data.find("positions") == sensor_data.end())
                    IWBC_ERROR("ActiveWalk2 needs the positions");

                auto controller = std::dynamic_pointer_cast<inria_wbc::controllers::HumanoidPosTracker>(controller_);

                auto positions = sensor_data.at("positions");
                auto q_tsid = controller->q_tsid();
                for (int i = 0; i < positions.size(); i++) {
                    if (!(std::abs(positions(i, 0)) < 1e-10 && std::abs(q_tsid[i + 7]) > 1e-10))
                        q_tsid[i + 7] = positions(i, 0);
                }
                if ((q_tsid - controller->q_tsid()).norm() > error_posture_) {
                    controller->posture_task()->setReference(trajs::to_sample(q_tsid.tail(controller->robot()->na())));
                }

                Eigen::Vector2d cop = Eigen::Vector2d::Zero();
                if (controller->cop())
                    cop = controller->cop().value();

                // bool keep_sending_com_traj = true;

                lh_init_ = controller->get_se3_ref("lh");
                rh_init_ = controller->get_se3_ref("rh");
                lh_final_ = lh_init_;
                rh_final_ = rh_init_;
                lh_final_.translation() = controller->model_frame_pos(torso_link_name_).translation() + lh_delta_init_;
                rh_final_.translation() = controller->model_frame_pos(torso_link_name_).translation() + rh_delta_init_;
                controller->set_se3_ref(lh_final_, "lh");
                controller->set_se3_ref(rh_final_, "rh");

                //GO_TO_RF: PUT COM ON RF
                if (state_ == States::GO_TO_RF) {
                    if (begin_) {
                        std::cout << "GO_TO_RF" << std::endl;
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

                    // if (controller->cop() && activate_error_cop_)
                    // keep_sending_com_traj = keep_sending_com_traj && ((cop - com_final_.head(2)).norm() > error_cop_);

                    if (index_ < std::floor(traj_foot_duration_ / dt_)) {
                        auto ref = set_com_ref(com_init_, com_final_, traj_foot_duration_, index_);
                        lift_foot_up_ = ref(1) < com_foot_up_ || lift_foot_up_;
                        if (lift_foot_up_) {
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
                        // keep_sending_com_traj = true;
                        state_ = States::MOVE_LF_FORWARD;
                        begin_ = true;
                        lift_foot_up_ = false;
                    }
                }

                //LIFT_DOWN_LF: PUT LEFT FOOT ON THE GROUND
                if (state_ == States::LIFT_DOWN_LF) {

                    if (begin_) {
                        std::cout << "LIFT_DOWN_LF" << std::endl;
                        lf_init_ = controller->model_frame_pos(left_sole_name_);
                        lf_final_ = lf_init_;
                        lf_final_.translation()(2) = 0.0;
                        begin_ = false;
                        index_ = 0;
                    }

                    // keep_sending_com_traj = keep_sending_com_traj && sensor_data.at("lf_force")(2) < force_treshold_;

                    if (index_ < std::floor(transition_duration_ / dt_)) {
                        set_se3_ref(lf_init_, lf_final_, "lf_sole", "contact_lfoot", transition_duration_, index_);
                        index_++;
                    }
                    else {
                        if (remove_contacts_) {
                            controller->add_contact("contact_lfoot");
                        }

                        if (cycle_count_ == num_cycles_) {
                            next_state_ = States::GO_TO_MIDDLE;
                            state_ = States::GO_TO_MIDDLE;
                        }
                        else {
                            state_ = States::GO_TO_MIDDLE;
                            next_state_ = States::GO_TO_LF;
                        }
                        // keep_sending_com_traj = true;
                        begin_ = true;
                    }
                }
                
                //MOVE_RF_FORWARD: MOVE RIGHT FOOT AT SAME HEIGHT ABOVE THE GROUND
                if (state_ == States::MOVE_RF_FORWARD) {

                    if (begin_) {
                        std::cout << "MOVE_RF_FORWARD" << std::endl;
                        rf_init_ = controller->model_frame_pos(right_sole_name_);
                        rf_final_ = rf_init_;
                        if (first_step_ && cycle_count_ != num_cycles_)
                            rf_final_.translation()(1) -= step_lateral_;
                        rf_final_.translation()(0) += first_step_ ? 2 * step_length_ : 2 * step_length_;
                        begin_ = false;
                        index_ = 0;
                    }

                    // keep_sending_com_traj = keep_sending_com_traj && sensor_data.at("lf_force")(2) < force_treshold_;

                    if (index_ < std::floor(transition_duration_ / dt_)) {
                        set_se3_ref(rf_init_, rf_final_, "rf_sole", "contact_rfoot", transition_duration_, index_);
                        index_++;
                    }
                    else {
                        state_ = States::LIFT_DOWN_RF;
                        // keep_sending_com_traj = true;
                        begin_ = true;
                    }
                }
                
                //MOVE_LF_FORWARD: MOVE LEFT FOOT AT SAME HEIGHT ABOVE THE GROUND
                if (state_ == States::MOVE_LF_FORWARD) {

                    if (begin_) {
                        std::cout << "MOVE_LF_FORWARD" << std::endl;
                        lf_init_ = controller->model_frame_pos(left_sole_name_);
                        lf_final_ = lf_init_;
                        if (first_step_ && cycle_count_ != num_cycles_)
                            lf_final_.translation()(1) += step_lateral_;
                        lf_final_.translation()(0) += first_step_ ? step_length_ : 2 * step_length_;
                        begin_ = false;
                        index_ = 0;
                    }

                    // keep_sending_com_traj = keep_sending_com_traj && sensor_data.at("lf_force")(2) < force_treshold_;

                    if (index_ < std::floor(transition_duration_ / dt_)) {
                        set_se3_ref(lf_init_, lf_final_, "lf_sole", "contact_lfoot", transition_duration_, index_);
                        index_++;
                    }
                    else {
                        state_ = States::LIFT_DOWN_LF;
                        // keep_sending_com_traj = true;
                        begin_ = true;
                    }
                }

                //GO_TO_MIDDLE: PUT COM BETWEEN THE TWO FEET
                if (state_ == States::GO_TO_MIDDLE) {
                    if (begin_) {
                        std::cout << "GO_TO_MIDDLE" << std::endl;
                        com_init_ = controller->com();
                        com_final_ = com_init_;
                        com_final_.head(2) = 0.5 * (controller->model_frame_pos(left_sole_name_).translation().head(2) + controller->model_frame_pos(right_sole_name_).translation().head(2));
                        begin_ = false;
                        index_ = 0;
                    }

                    if (index_ < std::floor(transition_duration_ / dt_)) {
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
                        std::cout << "GO_TO_LF" << std::endl;
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

                    // if (controller->cop() && activate_error_cop_) {
                    //     // std::cout << (cop - com_final_.head(2)).norm() << std::endl;
                    //     keep_sending_com_traj = keep_sending_com_traj && ((cop - com_final_.head(2)).norm() > error_cop_);
                    // }

                    if (index_ < std::floor(traj_foot_duration_ / dt_)) {
                        auto ref = set_com_ref(com_init_, com_final_, traj_foot_duration_, index_);
                        lift_foot_up_ = ref(1) > com_foot_up_ || lift_foot_up_;
                        if (lift_foot_up_) {
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
                        // keep_sending_com_traj = true;
                        state_ = States::MOVE_RF_FORWARD;
                        begin_ = true;
                        lift_foot_up_ = false;
                    }
                }

                //LIFT_DOWN_RF: PUT RIGHT FOOT ON THE GROUND
                if (state_ == States::LIFT_DOWN_RF) {
                    if (begin_) {
                        rf_init_ = controller->model_frame_pos(right_sole_name_);
                        rf_final_ = rf_init_;
                        if (first_step_ && cycle_count_ != num_cycles_) {
                            first_step_ = false;
                        }
                        rf_final_.translation()(2) = 0.0;
                        begin_ = false;
                        index_ = 0;
                        if (first_step_)
                            first_step_ = false;
                    }

                    // keep_sending_com_traj = keep_sending_com_traj && sensor_data.at("rf_force")(2) < force_treshold_;

                    if (index_ < std::floor(transition_duration_ / dt_)) {
                        set_se3_ref(rf_init_, rf_final_, "rf_sole", "contact_rfoot", transition_duration_, index_);
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
                        // keep_sending_com_traj = true;
                    }
                }

                // Eigen::Matrix<double, 6, 1> left_fref, right_fref;
                // left_fref.setZero();
                // right_fref.setZero();

                // std::map<std::string, pinocchio::SE3> contact_se3_ref;
                // auto ac = controller->activated_contacts();
                // for (auto& contact_name : ac) {
                //     pinocchio::SE3 se3;
                //     auto contact_ext = std::dynamic_pointer_cast<tsid::contacts::Contact6dExt>(controller->contact(contact_name));
                //     auto contact_pos = contact_ext->getMotionTask().getReference().getValue();
                //     tsid::math::vectorToSE3(contact_pos, se3);
                //     contact_se3_ref[contact_name] = se3;
                // }

                auto zmp = stabilizer::com_to_zmp(controller->get_full_com_ref());
                Eigen::Vector3d zmp_ref = controller->get_com_ref();
                zmp_ref(2) = 0.0;
                if (controller->has_task("cop"))
                    controller->set_cop_ref(zmp_ref, "cop");
                // auto alpha = stabilizer::zmp_distributor(controller->pinocchio_total_model_mass(), zmp, contact_se3_ref, controller->activated_contacts(), left_fref, right_fref);

                // if (state_ == States::GO_TO_LF) {

                //     auto lcop = compute_foot_cop(controller->get_cop_ref("cop_rfoot").head(2), zmp, right_fref(2), left_fref(2));
                //     Eigen::Vector3d lcop_ref = Eigen::Vector3d::Zero();
                //     lcop_ref.head(2) = lcop;
                //     controller->set_cop_ref(lcop_ref, "cop_lfoot");
                // }
                // if (state_ == States::GO_TO_RF) {
                //     auto rcop = compute_foot_cop(controller->get_cop_ref("cop_lfoot").head(2), zmp, left_fref(2), right_fref(2));
                //     Eigen::Vector3d rcop_ref = Eigen::Vector3d::Zero();
                //     rcop_ref.head(2) = rcop;
                //     controller->set_cop_ref(rcop_ref, "cop_rfoot");
                // }
                // std::cout << "lcop " << controller->get_cop_ref("cop_lfoot").head(2).transpose() << " rcop " << controller->get_cop_ref("cop_rfoot").head(2).transpose() << " zmp " << zmp.transpose() << std::endl;
                // // controller->set_cop_ref(controller->get_se3_ref("lf_sole").translation(), "cop_lfoot");

                // if (keep_sending_com_traj && !controller->is_model_colliding())
                //     controller->set_send_cmd(true);
                // if (!keep_sending_com_traj) {
                //     if (k++ % 6 == 0)
                //         file_ << "stop" << std::endl;
                //     controller->set_send_cmd(false);
                // }

                controller->update(sensor_data);
            }
        } // namespace humanoid
    } // namespace behaviors
} // namespace inria_wbc
