#include "inria_wbc/behaviors/ex_behavior.hpp"

namespace inria_wbc {
    namespace behaviors {
        static Register<ExBehavior> __ex_behavior("ex-behavior");

        ExBehavior::ExBehavior(const controller_ptr_t& controller, const YAML::Node& config) : Behavior(controller, config)
        {
            //////////////////// DEFINE COM TRAJECTORIES  //////////////////////////////////////
            traj_selector_ = 0;
            auto tracker = std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller_);
            IWBC_ASSERT(tracker, "we need a pos tracker here");

            auto lh_init = tracker->get_se3_ref("lh");
            
            YAML::Node c = IWBC_CHECK(config["BEHAVIOR"]);
            trajectory_duration_ = IWBC_CHECK(c["trajectory_duration"].as<float>());
            motion_size_ = IWBC_CHECK(c["motion_size"].as<float>());

            auto lh_final = lh_init;
            lh_final.translation()(2) += motion_size_;

            trajectories_.push_back(trajectory_handler::compute_traj(lh_init, lh_final, controller_->dt(), trajectory_duration_));
            trajectories_.push_back(trajectory_handler::compute_traj(lh_final, lh_init, controller_->dt(), trajectory_duration_));
            current_trajectory_ = trajectories_[traj_selector_];
        }

        void ExBehavior::update(const controllers::SensorData& sensor_data)
        {
            auto ref = current_trajectory_[time_];
            std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_se3_ref(ref, "lh");

            controller_->update(sensor_data);
            time_++;
            if (time_ == current_trajectory_.size()) {
                time_ = 0;
                traj_selector_ = ++traj_selector_ % trajectories_.size();
                current_trajectory_ = trajectories_[traj_selector_];
            }
        }

    } // namespace behaviors
} // namespace inria_wbc