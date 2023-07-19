#ifndef IWBC_BEHAVIOR_HUMANOID_FOLLOW_TRACKERS_HPP
#define IWBC_BEHAVIOR_HUMANOID_FOLLOW_TRACKERS_HPP

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
                virtual ~FollowTrackers() {};
                void update(const controllers::SensorData& sensor_data = {}) override;
                std::string behavior_type() const override { return controllers::behavior_types::DOUBLE_SUPPORT; };

                void update_trajectories(const Eigen::Vector3d& target_right_pos,const Eigen::Vector3d& target_left_pos,
                                            const Eigen::Matrix3d& target_right_rot,const Eigen::Matrix3d& target_left_rot);

                Eigen::Vector3d get_rh_pos(){return _rh_current_task.translation();}
                Eigen::Vector3d get_lh_pos(){return _lh_current_task.translation();}
                Eigen::Vector3d get_init_right(){return _rh_init_pos;}
                Eigen::Vector3d get_init_left(){return _lh_init_pos;}
                Eigen::Matrix3d get_init_rot_right(){return _rh_init_rot;}
                Eigen::Matrix3d get_init_rot_left(){return _lh_init_rot;}
                Eigen::Matrix3d get_abs_rot(){return _abs_rot;}

            private:
            //target positions
                Eigen::Vector3d _rh_target_pos;
                Eigen::Vector3d _lh_target_pos;

            //current positions
                Eigen::Vector3d _rh_current_pos;
                Eigen::Vector3d _lh_current_pos;

            //initial positions
                Eigen::Vector3d _rh_init_pos;
                Eigen::Vector3d _lh_init_pos;

            //target rotations
                Eigen::Matrix3d _rh_target_rot;
                Eigen::Matrix3d _lh_target_rot;

            //current rotations
                Eigen::Matrix3d _rh_current_rot;
                Eigen::Matrix3d _lh_current_rot;

            //initial rotations
                Eigen::Matrix3d _rh_init_rot;
                Eigen::Matrix3d _lh_init_rot;
                Eigen::Matrix3d _abs_rot;

            //current tasks
                pinocchio::SE3 _rh_current_task;
                pinocchio::SE3 _lh_current_task;
            
            //target tasks
                pinocchio::SE3 _rh_target_task;
                pinocchio::SE3 _lh_target_task;

            //optimized trajectories
                std::vector<pinocchio::SE3> _trajectory_right;
                std::vector<Eigen::VectorXd> _trajectory_right_d;
                std::vector<Eigen::VectorXd> _trajectory_right_dd;
                std::vector<pinocchio::SE3> _trajectory_left;
                std::vector<Eigen::VectorXd> _trajectory_left_d;
                std::vector<Eigen::VectorXd> _trajectory_left_dd;

                std::vector<pinocchio::SE3> _last_trajectory_right;
                std::vector<pinocchio::SE3> _last_trajectory_left;

                int _time = 0;

                std::vector<std::string> task_names_;
                float trajectory_duration_;
                bool absolute_;
            };
            
            
        } // namespace humanoid
        
    } // namespace behaviors   
    
} // namespace inria_wbc

#endif
