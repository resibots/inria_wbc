#include "inria_wbc/behaviors/generic/cartesian_traj.hpp"

namespace inria_wbc {
    namespace behaviors {
        namespace generic {
            static Register<CartesianTraj> __generic_cartesian_trajectory("generic::cartesian_traj");

            CartesianTraj::CartesianTraj(const controller_ptr_t& controller, const YAML::Node& config) : Behavior(controller, config)
            {
                auto c = IWBC_CHECK(config["BEHAVIOR"]);
                loop_ = IWBC_CHECK(c["loop"].as<bool>());
                auto yaml_traj = IWBC_CHECK(c["trajectories"].as<std::string>());
                auto traj_yaml_path = controller->base_path() + "/" + yaml_traj;
                traj_loader_ = std::make_shared<trajs::Loader>(traj_yaml_path);
                scale_ = IWBC_CHECK(c["scale"].as<float>());
                step_ = 1;
            }

            void CartesianTraj::update(const controllers::SensorData& sensor_data)
            {
                auto controller = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_);
                // tasks
                for (auto& task : traj_loader_->ref_names()) {
                    auto ref = traj_loader_->task_ref(task, time_);
                    auto ref_scaled = ref;
                    ref_scaled.translation() = ref.translation() * scale_;
                    if (controller->verbose())
                        std::cout << "task:" << task << " : " << ref_scaled.translation().transpose() << std::endl;
                    controller->set_se3_ref(ref_scaled, task);
                }
                // com
                if (traj_loader_->has_com_refs()) {
                    auto ref = traj_loader_->com_ref(time_);
                    auto ref_scaled = ref;
                    ref_scaled = ref * scale_;
                    controller->set_com_ref(ref_scaled);
                }
                controller->update(sensor_data);

                time_ += step_;

                if (time_ >= traj_loader_->size() - 1)
                    step_ = -1;
                else if (time_ <= 0)
                    step_ = 1;
            }
        } // namespace generic
    } // namespace behaviors
} // namespace inria_wbc