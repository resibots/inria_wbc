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
                behavior_type_ = this->behavior_type();
                _rh_current_task = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->get_se3_ref(task_names_[1]);
                _lh_current_task = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->get_se3_ref(task_names_[0]);
                _abs_rot = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->get_se3_ref("head").rotation();
                trajectory_duration_ = 20*controller_->dt();

                //get current positions
                _rh_current_pos = _rh_current_task.translation();
                _lh_current_pos = _lh_current_task.translation();

                //get current rotations
                _rh_current_rot = _rh_current_task.rotation();
                _lh_current_rot = _lh_current_task.rotation();

                //rot by pi to follow the good wrist rotations, to avoid flipping by pi the real wrist
                // Eigen::Matrix3d rot_pi_rh = Eigen::AngleAxisd(M_PI,_rh_current_rot*Eigen::Vector3d::UnitZ()).toRotationMatrix();
                // _rh_current_rot = rot_pi_rh*_rh_current_rot;
                // _rh_current_task.rotation() = rot_pi_rh*_rh_current_task.rotation();

                //also initial btw
                _rh_init_pos = _rh_current_pos;
                _lh_init_pos = _lh_current_pos;
                _rh_init_rot = _rh_current_rot;
                _lh_init_rot = _lh_current_rot;

                //just to initialize the target pos and rot
                _rh_target_pos = _rh_current_pos;
                _lh_target_pos = _lh_current_pos;

                _rh_target_rot = _rh_current_rot;
                _lh_target_rot = _lh_current_rot;


                //same for the tasks
                _rh_target_task = _rh_current_task;
                _lh_target_task = _lh_current_task;

                //calculate the optimized trajectories (should be 0 here because targets and currents are the same)
                _trajectory_right = trajs::min_jerk_trajectory(_rh_current_task, _rh_target_task, controller_->dt(), trajectory_duration_);
                _trajectory_right_d = trajs::min_jerk_trajectory<trajs::d_order::FIRST>(_rh_current_task, _rh_target_task, controller_->dt(), trajectory_duration_);
                _trajectory_right_dd = trajs::min_jerk_trajectory<trajs::d_order::SECOND>(_rh_current_task, _rh_target_task, controller_->dt(), trajectory_duration_);

                _trajectory_left = trajs::min_jerk_trajectory(_lh_current_task, _lh_target_task, controller_->dt(), trajectory_duration_);
                _trajectory_left_d = trajs::min_jerk_trajectory<trajs::d_order::FIRST>(_lh_current_task, _lh_target_task, controller_->dt(), trajectory_duration_);
                _trajectory_left_dd = trajs::min_jerk_trajectory<trajs::d_order::SECOND>(_lh_current_task, _lh_target_task, controller_->dt(), trajectory_duration_);


            }


            void FollowTrackers::update_trajectories(const Eigen::Vector3d& target_right_pos,const Eigen::Vector3d& target_left_pos,
                                        const Eigen::Matrix3d& target_right_rot,const Eigen::Matrix3d& target_left_rot){
                
                // get target pos
                _rh_target_task.translation() = target_right_pos;
                _lh_target_task.translation() = target_left_pos;

                //get target rot
                _rh_target_task.rotation() = target_right_rot;
                _lh_target_task.rotation() = target_left_rot;

                //calculate the optimized trajectories
                _trajectory_right = trajs::min_jerk_trajectory(_rh_current_task, _rh_target_task, controller_->dt(), trajectory_duration_);
                _trajectory_right_d = trajs::min_jerk_trajectory<trajs::d_order::FIRST>(_rh_current_task, _rh_target_task, controller_->dt(), trajectory_duration_);
                _trajectory_right_dd = trajs::min_jerk_trajectory<trajs::d_order::SECOND>(_rh_current_task, _rh_target_task, controller_->dt(), trajectory_duration_);

                _trajectory_left = trajs::min_jerk_trajectory(_lh_current_task, _lh_target_task, controller_->dt(), trajectory_duration_);
                _trajectory_left_d = trajs::min_jerk_trajectory<trajs::d_order::FIRST>(_lh_current_task, _lh_target_task, controller_->dt(), trajectory_duration_);
                _trajectory_left_dd = trajs::min_jerk_trajectory<trajs::d_order::SECOND>(_lh_current_task, _lh_target_task, controller_->dt(), trajectory_duration_);

            }

            void FollowTrackers::update(const controllers::SensorData& sensor_data){
                

                //set the ref for right hand
                auto ref_r = _trajectory_right[_time];
                Eigen::VectorXd ref_vec_r(12);
                tsid::math::SE3ToVector(ref_r, ref_vec_r);

                tsid::trajectories::TrajectorySample sample_ref_r(12,6);
                sample_ref_r.setValue(ref_vec_r);
                sample_ref_r.setDerivative( _trajectory_right_d[_time] );
                sample_ref_r.setSecondDerivative( _trajectory_right_dd[_time] );

                std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_se3_ref(sample_ref_r, task_names_[1]);
                /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                //set the ref for left hand
                auto ref_l = _trajectory_left[_time];
                Eigen::VectorXd ref_vec_l(12);
                tsid::math::SE3ToVector(ref_l, ref_vec_l);

                tsid::trajectories::TrajectorySample sample_ref_l(12,6);
                sample_ref_l.setValue(ref_vec_l);
                sample_ref_l.setDerivative( _trajectory_left_d[_time] );
                sample_ref_l.setSecondDerivative( _trajectory_left_dd[_time] );


                std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_se3_ref(sample_ref_l, task_names_[0]);
                /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                //update the controller's data
                controller_->update(sensor_data);

                //increment time, go to the next point of trajectory
                _time = (_time+1)%_trajectory_left.size(); //don't go over the trajectory vector's size

                //get current rh and lh tasks
                _rh_current_task = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->get_se3_ref(task_names_[1]);
                _lh_current_task = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->get_se3_ref(task_names_[0]);
                
                //rot by pi to follow the good wrist rotations, to avoid flipping by pi the real wrist
                // Eigen::Matrix3d rot_pi_rh = Eigen::AngleAxisd(M_PI,_rh_current_task.rotation()*Eigen::Vector3d::UnitZ()).toRotationMatrix();
                //_rh_current_task.rotation() = rot_pi_rh*_rh_current_task.rotation();
            }
            
        } // namespace humanoid
        
    } // namespace behaviors
    
} // namespace inria_wbc
