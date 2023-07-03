//include the corresponding hpp file
#include "inria_wbc/behaviors/generic/cartesian_sequential.hpp"

namespace inria_wbc{
    namespace behaviors{
        namespace generic{

            //Registers the behavior into the factory
            static Register<CartesianSequential> __talos_move_sequential("generic::cartesian_sequential"); 

            CartesianSequential::CartesianSequential(const controller_ptr_t& controller, const YAML::Node& config) : Behavior(controller, config),traj_selector_(0){

                //initialization of both the parameters and the controller using the data collected from the yaml file
                auto c = IWBC_CHECK(config["BEHAVIOR"]);
                loop_ = IWBC_CHECK(c["loop"].as<bool>());
                task_names_ = IWBC_CHECK(c["task_names"].as<std::vector<std::string>>());
                trajectory_duration_ = IWBC_CHECK(c["trajectory_duration"].as<float>());
                behavior_type_ = this->behavior_type();
                controller_->set_behavior_type(behavior_type_);
                absolute_ = IWBC_CHECK(c["absolute"].as<bool>());

                //get the points to achieve for both hands
                rh_targets = IWBC_CHECK(c["rh_targetpositions"].as<std::vector<std::vector<double>>>());
                lh_targets = IWBC_CHECK(c["lh_targetpositions"].as<std::vector<std::vector<double>>>());

                //get the rotations to make
                rh_rots = IWBC_CHECK(c["rh_targetrotations"].as<std::vector<std::vector<double>>>());
                lh_rots = IWBC_CHECK(c["lh_targetrotations"].as<std::vector<std::vector<double>>>());

                //std::cout << rh_targets.size() << " " << lh_targets.size() << std::endl;

                if (rh_targets.size() != lh_targets.size()) //looking if tasks are correctly defined
                {
                    IWBC_ERROR("error: right hand and left hand don't have the same number of tasks.");
                }

                //get the number of targets
                number_of_targets = rh_targets.size();

                //get initial positions of both right and left hands tasks
                auto task_init_right = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->get_se3_ref(task_names_[1]);
                auto task_init_left = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->get_se3_ref(task_names_[0]);

                //initialiazing final tasks for iteration ("for" loop below)
                auto task_final_right = task_init_right;
                auto task_final_left = task_init_left;
                

                //construction of two vectors containing the different positions of respectively right and left hands
                for (int i = 0;i < number_of_targets;i++){

                    //saving the old tasks before changing them
                    auto old_task_right = task_final_right;
                    auto old_task_left = task_final_left;

                    //taking care of the translations
                    if(rh_targets[i].size() == 3) //only works if 3D vectors are given
                    {
                        if (absolute_)
                            task_final_right.translation() = Eigen::Vector3d::Map(rh_targets[i].data()) + task_init_right.translation(); //translate to get the final position of the hand
                        else
                            task_final_right.translation() = Eigen::Vector3d::Map(rh_targets[i].data()) + old_task_right.translation();
                    }
                    if(lh_targets[i].size() == 3) //same
                    {
                        if (absolute_)
                            task_final_left.translation() = Eigen::Vector3d::Map(lh_targets[i].data()) + task_init_left.translation(); //same
                        else
                            task_final_left.translation() = Eigen::Vector3d::Map(lh_targets[i].data()) + old_task_left.translation(); 
                    }
                    /*bool w = old_task_right.translation() == task_final_right.translation();
                    std::cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa " << w << std::endl;*/

                    //doing the same for the rotations
                    if (rh_rots[i].size() == 3) //same than above
                    {
                        Eigen::Matrix3d rot = (Eigen::AngleAxisd(rh_rots[i][2], Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(rh_rots[i][1], Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(rh_rots[i][0], Eigen::Vector3d::UnitX())).toRotationMatrix();
                        if (absolute_){
                            task_final_right.rotation() = rot*task_init_right.rotation();
                        }
                        else{
                            task_final_right.rotation() = rot*old_task_right.rotation();
                        }
                    }

                    if (lh_rots[i].size() == 3) //same than above
                    {
                        Eigen::Matrix3d rot = (Eigen::AngleAxisd(lh_rots[i][2], Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(lh_rots[i][1], Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(lh_rots[i][0], Eigen::Vector3d::UnitX())).toRotationMatrix();
                        if (absolute_){
                            task_final_left.rotation() = rot*task_init_left.rotation();
                        }
                        else{
                            task_final_left.rotation() = rot*old_task_left.rotation();
                        }
                    }
                    

                    //calculating optimized right hand's trajectory from the (i-1)-th to the i-th point
                    trajectories_r_.push_back(trajs::min_jerk_trajectory(old_task_right, task_final_right, controller_->dt(), trajectory_duration_));
                    trajectories_d_r_.push_back(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(old_task_right, task_final_right, controller_->dt(), trajectory_duration_));
                    trajectories_dd_r_.push_back(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(old_task_right, task_final_right, controller_->dt(), trajectory_duration_));
                    
                    //doing the same for left hand
                    trajectories_l_.push_back(trajs::min_jerk_trajectory(old_task_left, task_final_left, controller_->dt(), trajectory_duration_));
                    trajectories_d_l_.push_back(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(old_task_left, task_final_left, controller_->dt(), trajectory_duration_));
                    trajectories_dd_l_.push_back(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(old_task_left, task_final_left, controller_->dt(), trajectory_duration_));

                }

                //considering the loop
                if (loop_)
                {
                    trajectories_r_.push_back(trajs::min_jerk_trajectory(task_final_right, task_init_right, controller_->dt(), trajectory_duration_));
                    trajectories_d_r_.push_back(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(task_final_right, task_init_right, controller_->dt(), trajectory_duration_));
                    trajectories_dd_r_.push_back(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(task_final_right, task_init_right, controller_->dt(), trajectory_duration_));
                    
                    trajectories_l_.push_back(trajs::min_jerk_trajectory(task_final_left, task_init_left, controller_->dt(), trajectory_duration_));
                    trajectories_d_l_.push_back(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(task_final_left, task_init_left, controller_->dt(), trajectory_duration_));
                    trajectories_dd_l_.push_back(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(task_final_left, task_init_left, controller_->dt(), trajectory_duration_));
                }

                std::cout << "number of point to achieve: " << trajectories_r_.size() << std::endl;

            } //end of CartesianSequential's constructor

            void CartesianSequential::update(const controllers::SensorData& sensor_data){

                /*std::cout << trajectories_l_[traj_selector_].size() << " "
                        << trajectories_d_l_[traj_selector_].size() << " "
                        << trajectories_dd_l_[traj_selector_].size() << " "
                        << std::endl;*/
                
                if (traj_selector_ < trajectories_r_.size()) {

                    /*for (int i = 0;i < trajectories_r_[traj_selector_][time_].translation().size();i++){
                        std::cout << "cccccccccccc " << trajectories_r_[traj_selector_][time_].translation()[i] << std::endl;
                    }
                    std::cout << std::endl;*/

                    auto ref_r = trajectories_r_[traj_selector_][time_];
                    Eigen::VectorXd ref_vec_r(12);
                    tsid::math::SE3ToVector(ref_r, ref_vec_r);

                    tsid::trajectories::TrajectorySample sample_ref_r(12,6);
                    sample_ref_r.setValue(ref_vec_r);
                    sample_ref_r.setDerivative( trajectories_d_r_[traj_selector_][time_] );
                    sample_ref_r.setSecondDerivative( trajectories_dd_r_[traj_selector_][time_] );

                    //set the ref for right hand
                    std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_se3_ref(sample_ref_r, task_names_[1]);
                }

                if (traj_selector_ < trajectories_l_.size()) {

                    auto ref_l = trajectories_l_[traj_selector_][time_];
                    Eigen::VectorXd ref_vec_l(12);
                    tsid::math::SE3ToVector(ref_l, ref_vec_l);

                    tsid::trajectories::TrajectorySample sample_ref_l(12,6);
                    sample_ref_l.setValue(ref_vec_l);
                    sample_ref_l.setDerivative( trajectories_d_l_[traj_selector_][time_] );
                    sample_ref_l.setSecondDerivative( trajectories_dd_l_[traj_selector_][time_] );

                    //set the ref for left hand
                    std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->set_se3_ref(sample_ref_l, task_names_[0]);
                }

                controller_->update(sensor_data);
                time_++;

                //actualize right and left hands respective trajectories considering new sensor information
                if (trajectories_r_.size() > 0){
                    if (time_ == trajectories_r_[traj_selector_].size()){
                        time_ = 0;
                        traj_selector_++;
                        if (loop_){
                            traj_selector_ = traj_selector_ % trajectories_r_.size();
                        }
                    }
                }
                
            }



        } //namespace generic
    } //namespace behaviors
} //namespace inria_wbc
