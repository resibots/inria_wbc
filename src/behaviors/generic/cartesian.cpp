#include "inria_wbc/behaviors/generic/cartesian.hpp"

namespace inria_wbc {
    namespace behaviors {
        namespace generic {
            static Register<Cartesian> __talos_move_arm("generic::cartesian");

            Cartesian::Cartesian(const controller_ptr_t& controller, const YAML::Node& config) : 
                Behavior(controller, config),
                traj_selector_(0)
            {
                auto c = IWBC_CHECK(config["BEHAVIOR"]);
                trajectory_duration_ = IWBC_CHECK(c["trajectory_duration"].as<float>());
                behavior_type_ = this->behavior_type();
                task_name_ =  IWBC_CHECK(c["task_name"].as<std::string>());
                auto t = IWBC_CHECK(c["relative_target"].as<std::vector<double>>());
                relative_target_ =  Eigen::Vector3d::Map(t.data());

                controller_->set_behavior_type(behavior_type_);
                auto lh_init = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->get_se3_ref(task_name_);
                auto lh_final = lh_init;
                lh_final.translation() = relative_target_ + lh_init.translation();
                std::cout<<"lh init:"<<lh_init.translation().transpose() << std::endl;
                std::cout<<"lh final:"<<lh_final.translation().transpose() << std::endl;

                trajectories_.push_back(trajectory_handler::compute_traj(lh_init, lh_final, controller_->dt(), trajectory_duration_));
                trajectories_.push_back(trajectory_handler::compute_traj(lh_final, lh_init, controller_->dt(), trajectory_duration_));
                current_trajectory_ = trajectories_[traj_selector_];
            }

            void Cartesian::update(const controllers::SensorData& sensor_data)
            {
                auto ref = current_trajectory_[time_];
                std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_se3_ref(ref, task_name_);

                controller_->update(sensor_data);
                time_++;
                if (time_ == current_trajectory_.size()) {
                    time_ = 0;
                    traj_selector_ = ++traj_selector_ % trajectories_.size();
                    current_trajectory_ = trajectories_[traj_selector_];
                }
            }
        } // namespace generic
    } // namespace behaviors
} // namespace inria_wbc