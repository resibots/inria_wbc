#include "inria_wbc/behaviors/franka_circular_cartesian.hpp"

namespace inria_wbc {
    namespace behaviors {

        static Register<CircCartTraj> __franka_circular_cartesian_trajectory("circular-cartesian-trajectory");

        CircCartTraj::CircCartTraj(const controller_ptr_t& controller) : Behavior(controller)
        {
            traj_index_=0.;
            YAML::Node config = YAML::LoadFile(controller_->params().sot_config_path)["BEHAVIOR"];
            pitch_angle_ = config["pitch_angle"].as<float>();
            radius_ = config["radius"].as<float>();
            xyz_offset_ <<
              config["x_0"].as<float>(),
              config["y_0"].as<float>(),
              config["z_0"].as<float>();
            traj_cycle_duration_ = config["traj_cycle_duration:"].as<float>();
            dt_ = controller_->dt();
            num_traj_steps_ = round( traj_cycle_duration_/ dt_);
            trajectory_.reserve(num_traj_steps_);

            for( int i =0; i < num_traj_steps_; ++i){
              float t = (float)i/(float)num_traj_steps_;

              pinocchio::SE3 ref_ee = func_traj( t );
              trajectory_.push_back( ref_ee);
            }

        }
        
        pinocchio::SE3 CircCartTraj::func_traj( const float t){

            pinocchio::SE3 ref_ee;
            const float beta = t*2*M_PI;

            Eigen::Vector3d ref_xyz;
            ref_xyz << 
              cos(beta) * radius_,
              sin(beta) * radius_,
              0.0;

            ref_ee.translation( ref_xyz + xyz_offset_);

            Eigen::Matrix3d rot_ee;
            const double phi =  0.;
            const double theta = pitch_angle_;
            const double psi =  beta + M_PI/2.; //~~ see if the offset is pi/2 or pi

            Eigen::Matrix3d R_x;
            R_x << 1., 0., 0.,
                   0., cos(phi), -sin(phi),
                   0., sin(phi), cos(phi);
            Eigen::Matrix3d R_y;
            R_y << cos(theta), 0., sin(theta),
                   0., 1., 0.,
                   -sin(theta), 0., cos(theta);
            Eigen::Matrix3d R_z;
            R_z << cos(psi), -sin(psi), 0.,
                   sin(psi), cos(psi), 0.,
                   0., 0., 1.;
            rot_ee = R_z * R_y * R_x;
            ref_ee.rotation(rot_ee); 

            return ref_ee;
        }

        void CircCartTraj::update(const controllers::SensorData& sensor_data)
        {
         
            if ( traj_index_ == num_traj_steps_){
              traj_index_ = 0;
            }

            std::static_pointer_cast<inria_wbc::controllers::FrankaPosTracker>(controller_)->set_task_ref(trajectory_[traj_index_],"ee");
            controller_->update(sensor_data);

            ++traj_index_;

            //set_task_ref( ref_ee, "ee");
            //pinocchio::SE3 ref_given  = get_se3_ref( "end_effector");
            //ref_given.disp_impl(std::cout);
            //Eigen::VectorXd ref_posture(9);
            //ref_posture << 0., M_PI / 4., 0., -M_PI / 4, 0., M_PI / 2., 0., 0., 0.;
            //set_posture_ref( ref_posture);
            //controller->set_task_ref(robot_->model().referenceConfigurations[m_ref_config],"posture");

        }

    } // namespace behaviors
} // namespace inria_wbc
