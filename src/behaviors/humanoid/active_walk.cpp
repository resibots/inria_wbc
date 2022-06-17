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

                behavior_type_ = this->behavior_type();
                controller_->set_behavior_type(behavior_type_);

                dt_ = controller_->dt();
                time_ = 0;

                state_ = States::INIT;

                com_init_ = h_controller->com();
                lh_init_ = h_controller->get_se3_ref("lh");
                rh_init_ = h_controller->get_se3_ref("rh");
                lf_init_ = h_controller->get_se3_ref("lf");
                rf_init_ = h_controller->get_se3_ref("rf");
                com_final_ = com_init_;
                lh_final_ = lh_init_;
                rh_final_ = rh_init_;
                lf_final_ = lf_init_;
                rf_final_ = rf_init_;

                left_ankle_name_ = h_controller->robot()->model().frames[h_controller->contact("contact_lfoot")->getMotionTask().frame_id()].name;
                right_ankle_name_ = h_controller->robot()->model().frames[h_controller->contact("contact_rfoot")->getMotionTask().frame_id()].name;
            }

            void ActiveWalk::update(const controllers::SensorData& sensor_data)
            {
                if (sensor_data.find("lf_force") == sensor_data.end())
                    IWBC_ERROR("ActiveWalk needs the LF force");

                if (sensor_data.find("rf_force") == sensor_data.end())
                    IWBC_ERROR("ActiveWalk needs the LF force");

                auto controller = std::dynamic_pointer_cast<inria_wbc::controllers::HumanoidPosTracker>(controller_);

                if (state_ == States::INIT) {
                    if (begin_) {
                        com_init_ = controller->com();
                        com_final_ = com_init_;
                        com_final_.head(2) = controller->get_se3_ref("rf").translation().head(2);
                        std::cout << "com_init_ " << com_init_.transpose() << std::endl;
                        std::cout << "com_final_ " << com_final_.transpose() << std::endl;
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
                        auto ref = trajs::min_jerk_trajectory(lf_init_, lf_final_, controller->dt(), traj_foot_duration_, index_);
                        sample_ref.setValue(ref_vec);
                        sample_ref.setDerivative(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(lf_init_, lf_final_, controller->dt(), traj_foot_duration_, index_));
                        sample_ref.setSecondDerivative(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(lf_init_, lf_final_, controller->dt(), traj_foot_duration_, index_));
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller)->set_se3_ref(ref, "lf");
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller)->set_contact_se3_ref(ref, "contact_lfoot");
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
                        lf_final_.translation()(0) += step_length_;
                        begin_ = false;
                        index_ = 0;
                    }
                    std::cout << sensor_data.at("lf_torque")(2) << std::endl;
                    if (index_ < std::floor(traj_foot_duration_ / controller->dt()) && sensor_data.at("lf_torque")(2) < force_treshold_) {
                        tsid::trajectories::TrajectorySample sample_ref(12, 6);
                        Eigen::VectorXd ref_vec(12);
                        tsid::math::SE3ToVector(trajs::min_jerk_trajectory(lf_init_, lf_final_, controller->dt(), traj_foot_duration_, index_), ref_vec);
                        auto ref = trajs::min_jerk_trajectory(lf_init_, lf_final_, controller->dt(), traj_foot_duration_, index_);
                        sample_ref.setValue(ref_vec);
                        sample_ref.setDerivative(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(lf_init_, lf_final_, controller->dt(), traj_foot_duration_, index_));
                        sample_ref.setSecondDerivative(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(lf_init_, lf_final_, controller->dt(), traj_foot_duration_, index_));
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller)->set_se3_ref(ref, "lf");
                        std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller)->set_contact_se3_ref(ref, "contact_lfoot");
                        index_++;
                    }
                    else {
                        controller->add_contact("contact_lfoot");
                        state_ = States::INIT;
                        begin_ = true;
                    }
                }
                controller->update(sensor_data);
            }

        } // namespace humanoid
    } // namespace behaviors
} // namespace inria_wbc
