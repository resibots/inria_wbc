#include "inria_wbc/behaviors/talos_clapping.hpp"

namespace inria_wbc {
    namespace behaviors {
        static Register<TalosClapping> __talos_clapping("talos-clapping");

        TalosClapping::TalosClapping(const controller_ptr_t& controller) : 
        Behavior(controller)
        {

            auto c = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_);
            auto lh_init = c->get_se3_ref("lh");
            auto rh_init = c->get_se3_ref("rh");

            YAML::Node config = YAML::LoadFile(controller_->params().sot_config_path)["BEHAVIOR"];
            trajectory_duration_ = config["trajectory_duration"].as<float>();
            motion_size_ = config["motion_size"].as<float>();

            auto lh_final = lh_init;
            auto rh_final = rh_init;

            lh_final.translation()(1) -= motion_size_;
            rh_final.translation()(1) += motion_size_;

            lh_trajs_.push_back(trajectory_handler::compute_traj(lh_init, lh_final, controller_->dt(), trajectory_duration_));
            lh_trajs_.push_back(trajectory_handler::compute_traj(lh_final, lh_init, controller_->dt(), trajectory_duration_));
            rh_trajs_.push_back(trajectory_handler::compute_traj(rh_init, rh_final, controller_->dt(), trajectory_duration_));
            rh_trajs_.push_back(trajectory_handler::compute_traj(rh_final, rh_init, controller_->dt(), trajectory_duration_));

        }

        void TalosClapping::update(const controllers::SensorData& sensor_data)
        {
            assert(lh_trajs_.size() == 2);
            assert(rh_trajs_.size() == 2);
            assert(rh_trajs_[current_traj_].size() == lh_trajs_[current_traj_].size());
            assert(rh_trajs_[current_traj_].size() > 0);
           
            auto controller = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_);
            

            controller->set_se3_ref(lh_trajs_[current_traj_][time_], "lh");
            controller->set_se3_ref(rh_trajs_[current_traj_][time_], "rh");
            controller->update(sensor_data);
            time_++;
            if (time_ == lh_trajs_[current_traj_].size()) {
                time_ = 0;
                current_traj_ = ++current_traj_ % lh_trajs_.size();
            }
        }
    } // namespace behaviors
} // namespace inria_wbc