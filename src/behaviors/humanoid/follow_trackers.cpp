#include "inria_wbc/behaviors/humanoid/follow_trackers.hpp"

namespace inria_wbc
{
    namespace behaviors
    {
        namespace humanoid
        {
            //Registers the behavior into the factory
            static Register<FollowTrackers> __talos_follow_trackers("humanoid::follow_trackers");
            //implement the behavior class

            FollowTrackers::FollowTrackers(const controller_ptr_t& controller, const YAML::Node& config) : Behavior(controller,config){

                //initialization of both the parameters and the controller using the data collected from the yaml file
                auto c = IWBC_CHECK(config["BEHAVIOR"]);
                task_names_ = IWBC_CHECK(c["task_names"].as<std::vector<std::string>>());
                trajectory_duration_ = IWBC_CHECK(c["trajectory_duration"].as<float>());
                behavior_type_ = this->behavior_type();
                controller_->set_behavior_type(behavior_type_);
                absolute_ = IWBC_CHECK(c["absolute"].as<bool>());

                //get current positions
                auto task_init_right = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->get_se3_ref(task_names_[1]);
                auto task_init_left = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->get_se3_ref(task_names_[0]);
                _rh_current_pos = task_init_right.translation();
                _lh_current_pos = task_init_left.translation();

                //get current rotations
                _rh_current_rot = task_init_right.rotation();
                _lh_current_rot = task_init_left.rotation();

                //just to initialize the target pos and rot
                _rh_target_pos = _rh_current_pos;
                _lh_target_pos = _lh_current_pos;

                _rh_target_rot = _rh_current_rot;
                _lh_target_rot = _lh_current_rot;

            }
        } // namespace humanoid
        
    } // namespace behaviors
    
} // namespace inria_wbc
