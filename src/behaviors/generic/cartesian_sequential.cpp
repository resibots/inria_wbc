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

                //get the points to achieve for both hands
                rh_targets = IWBC_CHECK(c["rh_targetpositions"].as<std::vector<std::vector<double>>>());
                lh_targets = IWBC_CHECK(c["lh_targetpositions"].as<std::vector<std::vector<double>>>());

                //std::cout << rh_targets.size() << " " << lh_targets.size() << std::endl;

                if (rh_targets.size() != lh_targets.size()) //looking if tasks are correctly defined
                {
                    IWBC_ERROR("error: right hand and left hand don't have the same number of tasks.");
                }

                //get the number of targets
                number_of_targets = rh_targets.size();

                //initialization of both right and left hands tasks
                auto task_init_right = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->get_se3_ref(task_names_[1]);
                auto task_init_left = std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)->get_se3_ref(task_names_[0]);

                //initialization of trajectory vectors. See their implementation below
                //right hand
                std::vector<std::vector<pinocchio::SE3>> trajectory_r;
                std::vector<std::vector<Eigen::VectorXd>> trajectory_d_r;
                std::vector<std::vector<Eigen::VectorXd>> trajectory_dd_r;
                //...........................

                //left hand
                std::vector<std::vector<pinocchio::SE3>> trajectory_l;
                std::vector<std::vector<Eigen::VectorXd>> trajectory_d_l;
                std::vector<std::vector<Eigen::VectorXd>> trajectory_dd_l;
                //...........................

                //initialiazing final tasks for iteration ("for" loop below)
                auto task_final_right = task_init_right;
                auto task_final_left = task_init_left;

                for (auto& element : rh_targets)
                {
                    for (int i = 0;i < element.size();i++){
                        std::cout << i <<"-ieme coordonnee droite: " << element[i] << std::endl;
                    }
                }

                for (auto& element : lh_targets)
                {
                    for (int i = 0;i < element.size();i++){
                        std::cout << i <<"-ieme coordonnee gauche: " << element[i] << std::endl;
                    }
                }
                


                //construction of two vectors containing the different positions of respectively right and left hands
                for (int i = 0;i < number_of_targets;i++){

                    //saving the old tasks before changing them
                    auto old_task_right = task_final_right;
                    auto old_task_left = task_final_left;

                    if(rh_targets[i].size() == 3) //only works if 3D vectors are given
                        task_final_right.translation() = Eigen::Vector3d::Map(rh_targets[i].data()) + old_task_right.translation(); //translate to get the final position of the hand

                    if(lh_targets[i].size() == 3) //same
                        task_final_left.translation() = Eigen::Vector3d::Map(lh_targets[i].data()) + old_task_left.translation(); //same

                    //calculating optimized right hand's trajectory from the (i-1)-th to the i-th point
                    trajectory_r.push_back(trajs::min_jerk_trajectory(old_task_right, task_final_right, controller_->dt(), trajectory_duration_));
                    trajectory_d_r.push_back(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(old_task_right, task_final_right, controller_->dt(), trajectory_duration_));
                    trajectory_dd_r.push_back(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(old_task_right, task_final_right, controller_->dt(), trajectory_duration_));
                    
                    //doing the same for left hand
                    trajectory_l.push_back(trajs::min_jerk_trajectory(old_task_left, task_final_left, controller_->dt(), trajectory_duration_));
                    trajectory_d_l.push_back(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(old_task_left, task_final_left, controller_->dt(), trajectory_duration_));
                    trajectory_dd_l.push_back(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(old_task_left, task_final_left, controller_->dt(), trajectory_duration_));

                }

                //considering the loop
                if (loop_)
                {

                    trajectory_r.push_back(trajs::min_jerk_trajectory(task_final_right, task_init_right, controller_->dt(), trajectory_duration_));
                    trajectory_d_r.push_back(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(task_final_right, task_init_right, controller_->dt(), trajectory_duration_));
                    trajectory_dd_r.push_back(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(task_final_right, task_init_right, controller_->dt(), trajectory_duration_));
                    
                    trajectory_l.push_back(trajs::min_jerk_trajectory(task_final_left, task_init_left, controller_->dt(), trajectory_duration_));
                    trajectory_d_l.push_back(trajs::min_jerk_trajectory<trajs::d_order::FIRST>(task_final_left, task_init_left, controller_->dt(), trajectory_duration_));
                    trajectory_dd_l.push_back(trajs::min_jerk_trajectory<trajs::d_order::SECOND>(task_final_left, task_init_left, controller_->dt(), trajectory_duration_));

                    
                }

                //stocking them
                trajectories_r_ = trajectory_r;
                trajectories_d_r_ = trajectory_d_r;
                trajectories_dd_r_ = trajectory_dd_r;

                trajectories_l_ = trajectory_l;
                trajectories_d_l_ = trajectory_d_l;
                trajectories_dd_l_ = trajectory_dd_l;

                std::cout << "number of point to achieve: " << trajectories_r_.size() << std::endl;

            } //end of CartesianSequential's constructor

            void CartesianSequential::update(const controllers::SensorData& sensor_data){

                /*std::cout << trajectories_l_[traj_selector_].size() << " "
                        << trajectories_d_l_[traj_selector_].size() << " "
                        << trajectories_dd_l_[traj_selector_].size() << " "
                        << std::endl;*/
                
                if (traj_selector_ < trajectories_r_.size()) {

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

