#include "inria_wbc/behaviors/circular_cartesian_trajectory.hpp"

namespace inria_wbc {
    namespace behaviors {

        static Register<CircularCartesianTrajectory> __circular_cartesian_trajectory("circular-cartesian-trajectory");

        CircularCartesianTrajectory::CircularCartesianTrajectory(const controller_ptr_t& controller, const YAML::Node& config):
        Behavior(controller, config)
        {
            traj_index_=0.;
            YAML::Node c = IWBC_CHECK(config["BEHAVIOR"]);

            pitch_angle_ = c["pitch_angle"].as<float>();
            radius_ = c["radius"].as<float>();
            auto t_0 = c["init_pos"].as<std::vector<double>>();
            xyz_offset_ = Eigen::Vector3d(t_0.data());

            traj_cycle_duration_ = c["traj_cycle_duration"].as<float>();
            dt_ = controller_->dt();
            num_traj_steps_ = round( traj_cycle_duration_/ dt_);
            trajectory_.reserve(num_traj_steps_);

            for( int i =0; i < num_traj_steps_; ++i){
              float t = (float)i/(float)num_traj_steps_;

              pinocchio::SE3 ref_ee = func_traj( t );
              trajectory_.push_back( ref_ee);
            }

        }

        pinocchio::SE3 CircularCartesianTrajectory::func_traj( const float t){

            pinocchio::SE3 ref_ee;
            const float beta = t*2*M_PI;

            Eigen::Vector3d ref_xyz;
            ref_xyz <<
              0.0,
              sin(beta) * radius_,
              cos(beta) * radius_;

            ref_ee.translation( ref_xyz + xyz_offset_);

            Eigen::Matrix3d rot_ee;
            const double phi =  M_PI;
            const double theta = pitch_angle_ + M_PI/2.;
            const double psi =  0.;

            rot_ee =  Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(theta,  Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX());
            ref_ee.rotation(rot_ee);

            return ref_ee;
        }

        void CircularCartesianTrajectory::update(const controllers::SensorData& sensor_data)
        {

            if ( traj_index_ == num_traj_steps_){
              traj_index_ = 0;
            }
            std::static_pointer_cast<inria_wbc::controllers::PosTracker>(controller_)
              ->set_se3_ref(trajectory_[traj_index_],"ee");

            controller_->update(sensor_data);
            ++traj_index_;
        }
    } // namespace behaviors
} // namespace inria_wbc
