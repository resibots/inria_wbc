#include "inria_wbc/behaviors/humanoid/move_com.hpp"

namespace inria_wbc {
    namespace behaviors {
        namespace humanoid {
            static Register<MoveCom> __talos_move_com("humanoid::move_com");

            MoveCom::MoveCom(const controller_ptr_t& controller, const YAML::Node& config) : Behavior(controller, config)
            {
                IWBC_ASSERT(std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller_) != 0, "Need a PosTracker for MovCom");
                auto c = IWBC_CHECK(config["BEHAVIOR"]);
                float trajectory_duration = IWBC_CHECK(c["trajectory_duration"].as<float>());
                behavior_type_ = this->behavior_type();
                controller_->set_behavior_type(behavior_type_);

                loop_ = IWBC_CHECK(c["loop"].as<bool>());
                auto targets = IWBC_CHECK(c["targets"].as<std::vector<std::vector<double>>>());
                auto mask = IWBC_CHECK(c["mask"].as<std::string>());
                auto absolute = IWBC_CHECK(c["absolute"].as<bool>());
                IWBC_ASSERT(mask.size() == 3, "The mask for the CoM should be 3-dimensional");


                Eigen::VectorXd task_init = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->get_com_ref();
                if (loop_)
                    if (absolute)
                        targets.push_back({task_init(0), task_init(1), task_init(2)});
                    else
                        targets.push_back({0., 0., 0.});
       

                Eigen::VectorXd start = task_init;
                for (size_t i = 0; i < targets.size(); ++i) {
                    IWBC_ASSERT(targets[i].size() == 3, "references need to be 3-dimensional");
                    Eigen::VectorXd end = task_init;
                    for (size_t j = 0; j < targets[i].size(); ++j)
                        if (mask[j] == '1')
                            end[j] = absolute ? targets[i][j] : targets[i][j] + task_init[j];
                    auto traj = trajs::min_jerk_trajectory(start, end, controller_->dt(), trajectory_duration);
                    auto traj_d = trajs::min_jerk_trajectory<trajs::d_order::FIRST>(start, end, controller_->dt(), trajectory_duration);
                    auto traj_dd = trajs::min_jerk_trajectory<trajs::d_order::SECOND>(start, end, controller_->dt(), trajectory_duration);
                    trajectory_.insert(trajectory_.end(), traj.begin(), traj.end());
                    trajectory_d_.insert(trajectory_d_.end(), traj_d.begin(), traj_d.end());
                    trajectory_dd_.insert(trajectory_dd_.end(), traj_dd.begin(), traj_dd.end());
                    start = end;
                }
            }

            void MoveCom::update(const controllers::SensorData& sensor_data)
            {
                tsid::trajectories::TrajectorySample sample_ref(3, 3);
                sample_ref.setValue(trajectory_[time_]);
                sample_ref.setDerivative(trajectory_d_[time_]);
                sample_ref.setSecondDerivative(trajectory_dd_[time_]);
                std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_com_ref(sample_ref);
                controller_->update(sensor_data);
                time_++;
                if (loop_)
                    time_ = time_ % trajectory_.size();
                else
                    time_ = std::min(time_, (int)  trajectory_.size() - 1);
            }
        } // namespace humanoid
    } // namespace behaviors
} // namespace inria_wbc