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
                step_height_ = IWBC_CHECK(c["step_height"].as<float>());
                step_length_ = IWBC_CHECK(c["step_length"].as<float>());
                force_treshold_ = IWBC_CHECK(c["force_treshold"].as<float>());
                remove_contacts_ = IWBC_CHECK(c["remove_contacts"].as<bool>());
                num_cycles_ = IWBC_CHECK(c["num_cycles"].as<int>());
                go_to_middle_ = IWBC_CHECK(c["go_to_middle"].as<bool>());
                com_percentage_ = IWBC_CHECK(c["com_percentage"].as<float>());
                if (com_percentage_ >= 1.0 || com_percentage_ <= 0.0)
                    IWBC_ERROR("com_percentage_ should be between 0.0 and 1.0, it is ", com_percentage_);

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
            }

            void ActiveWalk::update(const controllers::SensorData& sensor_data)
            {
                if (sensor_data.find("lf_force") == sensor_data.end())
                    IWBC_ERROR("ActiveWalk needs the LF force");

                if (sensor_data.find("rf_force") == sensor_data.end())
                    IWBC_ERROR("ActiveWalk needs the LF force");

                auto controller = std::dynamic_pointer_cast<inria_wbc::controllers::HumanoidPosTracker>(controller_);

                if (state_ == States::GO_TO_MIDDLE) {
                    if (begin_) {
                        com_init_ = controller->com();
                        com_final_ = com_init_;
                        com_final_.head(2) = 0.5 * (controller->get_se3_ref("rf").translation().head(2) + controller->get_se3_ref("lf").translation().head(2));
                        begin_ = false;
                        index_ = 0;
                    }

                    if (index_ < std::floor(traj_com_duration_ / controller->dt())) {
                        tsid::trajectories::TrajectorySample sample_ref(3, 3);
                        sample_ref.setValue(trajs::min_jerk_trajectory(com_init_, com_final_, controller->dt(), traj_com_duration_, index_));
                        sample_ref.setDerivative(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(com_init_, com_final_, controller->dt(), traj_com_duration_, index_));
                        sample_ref.setSecondDerivative(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(com_init_, com_final_, controller->dt(), traj_com_duration_, index_));
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller)->set_com_ref(trajs::min_jerk_trajectory(com_init_, com_final_, controller->dt(), traj_com_duration_, index_));
                        index_++;
                    }
                    else {
                        state_ = next_state_;
                        begin_ = true;
                    }
                }

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

                    if (index_ < std::floor(traj_com_duration_ / controller->dt())) {
                        tsid::trajectories::TrajectorySample sample_ref(3, 3);
                        sample_ref.setValue(trajs::min_jerk_trajectory(com_init_, com_final_, controller->dt(), traj_com_duration_, index_));
                        sample_ref.setDerivative(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(com_init_, com_final_, controller->dt(), traj_com_duration_, index_));
                        sample_ref.setSecondDerivative(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(com_init_, com_final_, controller->dt(), traj_com_duration_, index_));
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller)->set_com_ref(trajs::min_jerk_trajectory(com_init_, com_final_, controller->dt(), traj_com_duration_, index_));
                        index_++;
                    }
                    else {
                        state_ = States::LIFT_UP_LF;
                        begin_ = true;
                    }
                }

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

                    if (index_ < std::floor(traj_foot_duration_ / controller->dt())) {
                        tsid::trajectories::TrajectorySample sample_ref(12, 6);
                        Eigen::VectorXd ref_vec(12);
                        tsid::math::SE3ToVector(trajs::min_jerk_trajectory(lf_init_, lf_final_, controller->dt(), traj_foot_duration_, index_), ref_vec);
                        sample_ref.setValue(ref_vec);
                        sample_ref.setDerivative(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(lf_init_, lf_final_, controller->dt(), traj_foot_duration_, index_));
                        sample_ref.setSecondDerivative(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(lf_init_, lf_final_, controller->dt(), traj_foot_duration_, index_));
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller)->set_se3_ref(sample_ref, "lf");
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller)->set_contact_se3_ref(sample_ref, "contact_lfoot");
                        index_++;
                    }
                    else {
                        state_ = States::LIFT_DOWN_LF;
                        begin_ = true;
                    }
                }

                if (state_ == States::LIFT_DOWN_LF) {

                    if (begin_) {
                        lf_init_ = controller->model_frame_pos(left_ankle_name_);
                        lf_final_ = lf_init_;
                        lf_final_.translation()(2) -= step_height_;
                        lf_final_.translation()(0) += first_step_ ? step_length_ : 2 * step_length_;
                        lh_init_ = controller->robot()->framePosition(controller->tsid()->data(), controller->task<tsid::tasks::TaskSE3Equality>("lh")->frame_id());
                        lh_final_ = lh_init_;
                        lh_final_.translation()(0) += first_step_ ? step_length_ : 2 * step_length_;
                        begin_ = false;
                        index_ = 0;
                        if (first_step_)
                            first_step_ = false;
                    }

                    if (index_ < std::floor(traj_foot_duration_ / controller->dt()) && sensor_data.at("lf_force")(2) < force_treshold_) {
                        tsid::trajectories::TrajectorySample sample_ref(12, 6);
                        Eigen::VectorXd ref_vec(12);
                        tsid::math::SE3ToVector(trajs::min_jerk_trajectory(lf_init_, lf_final_, controller->dt(), traj_foot_duration_, index_), ref_vec);
                        sample_ref.setValue(ref_vec);
                        sample_ref.setDerivative(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(lf_init_, lf_final_, controller->dt(), traj_foot_duration_, index_));
                        sample_ref.setSecondDerivative(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(lf_init_, lf_final_, controller->dt(), traj_foot_duration_, index_));
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller)->set_se3_ref(sample_ref, "lf");
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller)->set_contact_se3_ref(sample_ref, "contact_lfoot");

                        tsid::trajectories::TrajectorySample sample_ref_hand(12, 6);
                        Eigen::VectorXd ref_vec_hand(12);
                        tsid::math::SE3ToVector(trajs::min_jerk_trajectory(lh_init_, lh_final_, controller->dt(), traj_foot_duration_, index_), ref_vec_hand);
                        sample_ref_hand.setValue(ref_vec_hand);
                        sample_ref_hand.setDerivative(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(lh_init_, lh_final_, controller->dt(), traj_foot_duration_, index_));
                        sample_ref_hand.setSecondDerivative(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(lh_init_, lh_final_, controller->dt(), traj_foot_duration_, index_));
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller)->set_se3_ref(sample_ref_hand, "lh");
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

                    if (index_ < std::floor(traj_com_duration_ / controller->dt())) {
                        tsid::trajectories::TrajectorySample sample_ref(3, 3);
                        sample_ref.setValue(trajs::min_jerk_trajectory(com_init_, com_final_, controller->dt(), traj_com_duration_, index_));
                        sample_ref.setDerivative(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(com_init_, com_final_, controller->dt(), traj_com_duration_, index_));
                        sample_ref.setSecondDerivative(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(com_init_, com_final_, controller->dt(), traj_com_duration_, index_));
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller)->set_com_ref(sample_ref);
                        index_++;
                    }
                    else {
                        state_ = States::LIFT_UP_RF;
                        begin_ = true;
                    }
                }

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

                    if (index_ < std::floor(traj_foot_duration_ / controller->dt())) {
                        tsid::trajectories::TrajectorySample sample_ref(12, 6);
                        Eigen::VectorXd ref_vec(12);
                        tsid::math::SE3ToVector(trajs::min_jerk_trajectory(rf_init_, rf_final_, controller->dt(), traj_foot_duration_, index_), ref_vec);
                        auto ref = trajs::min_jerk_trajectory(rf_init_, rf_final_, controller->dt(), traj_foot_duration_, index_);
                        sample_ref.setValue(ref_vec);
                        sample_ref.setDerivative(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(rf_init_, rf_final_, controller->dt(), traj_foot_duration_, index_));
                        sample_ref.setSecondDerivative(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(rf_init_, rf_final_, controller->dt(), traj_foot_duration_, index_));
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller)->set_se3_ref(sample_ref, "rf");
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller)->set_contact_se3_ref(sample_ref, "contact_rfoot");
                        index_++;
                    }
                    else {
                        state_ = States::LIFT_DOWN_RF;
                        begin_ = true;
                    }
                }

                if (state_ == States::LIFT_DOWN_RF) {

                    if (begin_) {
                        rf_init_ = controller->model_frame_pos(right_ankle_name_);
                        rf_final_ = rf_init_;
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
                    if (index_ < std::floor(traj_foot_duration_ / controller->dt()) && sensor_data.at("rf_force")(2) < force_treshold_) {
                        tsid::trajectories::TrajectorySample sample_ref(12, 6);
                        Eigen::VectorXd ref_vec(12);
                        tsid::math::SE3ToVector(trajs::min_jerk_trajectory(rf_init_, rf_final_, controller->dt(), traj_foot_duration_, index_), ref_vec);
                        auto ref = trajs::min_jerk_trajectory(rf_init_, rf_final_, controller->dt(), traj_foot_duration_, index_);
                        sample_ref.setValue(ref_vec);
                        sample_ref.setDerivative(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(rf_init_, rf_final_, controller->dt(), traj_foot_duration_, index_));
                        sample_ref.setSecondDerivative(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(rf_init_, rf_final_, controller->dt(), traj_foot_duration_, index_));
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller)->set_se3_ref(sample_ref, "rf");
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller)->set_contact_se3_ref(sample_ref, "contact_rfoot");

                        tsid::trajectories::TrajectorySample sample_ref_hand(12, 6);
                        Eigen::VectorXd ref_vec_hand(12);
                        tsid::math::SE3ToVector(trajs::min_jerk_trajectory(rh_init_, rh_final_, controller->dt(), traj_foot_duration_, index_), ref_vec_hand);
                        sample_ref_hand.setValue(ref_vec_hand);
                        sample_ref_hand.setDerivative(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(rh_init_, rh_final_, controller->dt(), traj_foot_duration_, index_));
                        sample_ref_hand.setSecondDerivative(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(rh_init_, rh_final_, controller->dt(), traj_foot_duration_, index_));
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller)->set_se3_ref(sample_ref_hand, "rh");
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
