//includes
#include <iostream>
#include <inria_wbc/behaviors/behavior.hpp>
#include <inria_wbc/controllers/pos_tracker.hpp>
#include <inria_wbc/trajs/trajectory_generator.hpp>
#include <inria_wbc/trajs/loader.hpp>

//code
namespace inria_wbc
{
    namespace behaviors
    {
        namespace humanoid
        {
            class FollowTrackers: public Behavior //behavior which makes the talos robot follow user's instructions
            {

            public:
                FollowTrackers(const controller_ptr_t& controller, const YAML::Node& config);
                ~FollowTrackers();
                void update(const controllers::SensorData& sensor_data = {}) override;
                std::string behavior_type() const override { return controllers::behavior_types::DOUBLE_SUPPORT; };

                void update_trajectories(Eigen::Vector3d& target_right,Eigen::Vector3d& target_left);
                Eigen::Vector3d get_rh_pos(){return _rh_target_pos;}
                Eigen::Vector3d get_lh_pos(){return _lh_target_pos;}

            private:
            //target positions
                Eigen::Vector3d _rh_target_pos;
                Eigen::Vector3d _lh_target_pos;

            //current positions
                Eigen::Vector3d _rh_current_pos;
                Eigen::Vector3d _lh_current_pos;

            //target rotations
                Eigen::Matrix3d _rh_target_rot;
                Eigen::Matrix3d _lh_target_rot;

            //current rotations
                Eigen::Matrix3d _rh_current_rot;
                Eigen::Matrix3d _lh_current_rot;

            //aimed trajectories
                std::vector<std::vector<pinocchio::SE3>> _trajectory_right;
                std::vector<std::vector<pinocchio::SE3>> _trajectory_left;

                int _time = 0;

                std::vector<std::string> task_names_;
                float trajectory_duration_;
                bool absolute_;
            };
            
            
        } // namespace humanoid
        
    } // namespace behaviors   
    
} // namespace inria_wbc
