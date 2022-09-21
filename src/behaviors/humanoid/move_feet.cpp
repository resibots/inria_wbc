#include "inria_wbc/behaviors/humanoid/move_feet.hpp"

namespace inria_wbc {
    namespace behaviors {
        namespace generic {
            static Register<MoveFeet> __talos_move_feet("humanoid::move-feet");

            MoveFeet::MoveFeet(const controller_ptr_t& controller, const YAML::Node& config) : Behavior(controller, config),
                                                                                               traj_selector_(0)
            {

                auto c = IWBC_CHECK(config["BEHAVIOR"]);
                trajectory_duration_ = IWBC_CHECK(c["trajectory_duration"].as<float>());
                behavior_type_ = this->behavior_type();
                controller_->set_behavior_type(behavior_type_);

                loop_ = IWBC_CHECK(c["loop"].as<bool>());
                lf_task_name_ = IWBC_CHECK(c["lf_task_name"].as<std::string>());
                rf_task_name_ = IWBC_CHECK(c["rf_task_name"].as<std::string>());
                lf_contact_name_ = IWBC_CHECK(c["lf_contact_name"].as<std::string>());
                rf_contact_name_ = IWBC_CHECK(c["rf_contact_name"].as<std::string>());

                IWBC_ASSERT(std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->has_task(lf_task_name_), "active_walk: a " + lf_task_name_ + " task is required");
                IWBC_ASSERT(std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->has_task(rf_task_name_), "active_walk: a " + rf_task_name_ + " task is required");
                IWBC_ASSERT(std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->has_contact(lf_contact_name_), "active_walk: a " + lf_contact_name_ + " task is required");
                IWBC_ASSERT(std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->has_contact(rf_contact_name_), "active_walk: a " + rf_contact_name_ + " task is required");

                auto lf_ts = IWBC_CHECK(c["lf_relative_targets_pos"].as<std::vector<std::vector<double>>>());
                auto lf_to = IWBC_CHECK(c["lf_relative_targets_rpy"].as<std::vector<std::vector<double>>>());

                auto rf_ts = IWBC_CHECK(c["rf_relative_targets_pos"].as<std::vector<std::vector<double>>>());
                auto rf_to = IWBC_CHECK(c["rf_relative_targets_rpy"].as<std::vector<std::vector<double>>>());

                if (lf_ts.size() != lf_to.size() || lf_ts.size() != rf_ts.size() || rf_ts.size() != rf_to.size())
                    IWBC_ERROR("lf_relative_targets_pos, lf_relative_targets_rpy, rf_relative_targets_pos, rf_relative_targets_rpy needs to have the same size");

                auto task_init = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->get_se3_ref(lf_task_name_);
                auto task_final = task_init;

                for (uint i = 0; i < lf_ts.size(); i++) {

                    if (lf_ts[i].size() == 3)
                        task_final.translation() = Eigen::Vector3d::Map(lf_ts[i].data()) + task_init.translation();

                    if (lf_to[i].size() == 3) {
                        Eigen::Matrix3d rot = (Eigen::AngleAxisd(lf_to[i][2], Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(lf_to[i][1], Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(lf_to[i][0], Eigen::Vector3d::UnitX()))
                                                  .toRotationMatrix();

                        task_final.rotation() = rot * task_init.rotation();
                    }

                    std::vector<std::vector<pinocchio::SE3>> trajectory;
                    std::vector<std::vector<Eigen::VectorXd>> trajectory_d;
                    std::vector<std::vector<Eigen::VectorXd>> trajectory_dd;

                    lf_trajectories_.push_back(trajs::min_jerk_trajectory(task_init, task_final, controller_->dt(), trajectory_duration_));
                    lf_trajectories_d_.push_back(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(task_init, task_final, controller_->dt(), trajectory_duration_));
                    lf_trajectories_dd_.push_back(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(task_init, task_final, controller_->dt(), trajectory_duration_));

                    task_init = task_final;
                }

                task_init = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->get_se3_ref(rf_task_name_);
                task_final = task_init;

                for (uint i = 0; i < rf_ts.size(); i++) {

                    if (rf_ts[i].size() == 3)
                        task_final.translation() = Eigen::Vector3d::Map(rf_ts[i].data()) + task_init.translation();

                    if (rf_to[i].size() == 3) {
                        Eigen::Matrix3d rot = (Eigen::AngleAxisd(rf_to[i][2], Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(rf_to[i][1], Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(rf_to[i][0], Eigen::Vector3d::UnitX()))
                                                  .toRotationMatrix();

                        task_final.rotation() = rot * task_init.rotation();
                    }

                    std::vector<std::vector<pinocchio::SE3>> trajectory;
                    std::vector<std::vector<Eigen::VectorXd>> trajectory_d;
                    std::vector<std::vector<Eigen::VectorXd>> trajectory_dd;

                    rf_trajectories_.push_back(trajs::min_jerk_trajectory(task_init, task_final, controller_->dt(), trajectory_duration_));
                    rf_trajectories_d_.push_back(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(task_init, task_final, controller_->dt(), trajectory_duration_));
                    rf_trajectories_dd_.push_back(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(task_init, task_final, controller_->dt(), trajectory_duration_));

                    task_init = task_final;
                }

                time_ = 0;
            }

            void MoveFeet::update(const controllers::SensorData& sensor_data)
            {

                if (traj_selector_ < lf_trajectories_.size()) {
                    auto ref = lf_trajectories_[traj_selector_][time_];
                    Eigen::VectorXd ref_vec(12);
                    tsid::math::SE3ToVector(ref, ref_vec);

                    tsid::trajectories::TrajectorySample sample_ref(12, 6);
                    sample_ref.setValue(ref_vec);
                    sample_ref.setDerivative(lf_trajectories_d_[traj_selector_][time_]);
                    sample_ref.setSecondDerivative(lf_trajectories_dd_[traj_selector_][time_]);

                    std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_se3_ref(sample_ref, lf_task_name_);
                    std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_contact_se3_ref(sample_ref, lf_contact_name_);
                }

                if (traj_selector_ < rf_trajectories_.size()) {

                    auto ref = rf_trajectories_[traj_selector_][time_];
                    Eigen::VectorXd ref_vec(12);
                    tsid::math::SE3ToVector(ref, ref_vec);

                    tsid::trajectories::TrajectorySample sample_ref(12, 6);
                    sample_ref.setValue(ref_vec);
                    sample_ref.setDerivative(rf_trajectories_d_[traj_selector_][time_]);
                    sample_ref.setSecondDerivative(rf_trajectories_dd_[traj_selector_][time_]);

                    std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_se3_ref(sample_ref, rf_task_name_);
                    std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_contact_se3_ref(sample_ref, rf_contact_name_);
                }

                controller_->update(sensor_data);
                ++time_;
                if (lf_trajectories_.size() > 0) {
                    if (time_ == lf_trajectories_[traj_selector_].size()) {
                        traj_selector_ = ++traj_selector_;
                        time_ = 0;
                    }
                }
            }
        } // namespace generic
    } // namespace behaviors
} // namespace inria_wbc