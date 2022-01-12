#ifndef IWBC_STABILIZER_HPP
#define IWBC_STABILIZER_HPP

#include <Eigen/Dense>
#include <string>
#include <unordered_map>

#include <pinocchio/spatial/se3.hpp>

#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/trajectories/trajectory-base.hpp>

namespace inria_wbc {
    namespace stabilizer {

        // creates a momentum in the direction where there is some com error
        // hence if you push the robot, the momentum will go against that push
        void momentum_com_admittance(
            double dt,
            const Eigen::VectorXd& p,
            const Eigen::VectorXd& d,
            const Eigen::Vector2d& cop_filtered,
            const tsid::trajectories::TrajectorySample& model_current_com,
            const tsid::trajectories::TrajectorySample& momentum_ref,
            tsid::trajectories::TrajectorySample& momentum_sample,
            float max_ref = 5);

        // creates a momentum to keep the torso in the correct orientation
        // if the imu data velocity goes in one direction we create a momentum to put back the torso where it should be
        void momentum_imu_admittance(
            double dt,
            const Eigen::VectorXd& p,
            const Eigen::VectorXd& d,
            const Eigen::Vector3d& imu_angular_vel,
            const Eigen::Vector3d& model_angular_vel,
            const tsid::trajectories::TrajectorySample& momentum_ref,
            tsid::trajectories::TrajectorySample& momentum_sample,
            float max_ref = 5);

        Eigen::Vector2d com_to_zmp(const tsid::trajectories::TrajectorySample& com_ref);
        tsid::trajectories::TrajectorySample data_to_sample(const tsid::InverseDynamicsFormulationAccForce::Data& data);

        //Make the measured cop match the current desired cop by changing the com ref
        void com_admittance(
            double dt, // controller dt
            const Eigen::VectorXd& p, // 6d proportional gains
            const Eigen::Vector2d& cop_filtered, //filtered cop estimation
            const tsid::trajectories::TrajectorySample& model_current_com, //pinocchio current com pos,vel,acc
            const tsid::trajectories::TrajectorySample& com_ref, //next com reference
            tsid::trajectories::TrajectorySample& se3_sample); //out : modified com ref to give to tsid

        /** Correct the roll and pitch angle of the ankle to force the foot cop to be in the middle of the foot
         *  see Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control
         *      by Stéphane Caron, Abderrahmane Kheddar, Olivier Tempier
        */
        void ankle_admittance(
            double dt, // controller dt
            const Eigen::VectorXd& p, // 6d proportional gains
            const Eigen::Vector2d& cop_foot, // filtered foot cop estimation (from the foot force/torque sensor)
            const pinocchio::SE3& model_current_foot, //current model foot position
            const tsid::trajectories::TrajectorySample& se3_sample_ref, // tsid ankle reference of the same foot
            const tsid::trajectories::TrajectorySample& contact_sample_ref, // tsid contact reference of the same foot
            tsid::trajectories::TrajectorySample& se3_sample, // out : modified ankle ref to give to tsid
            tsid::trajectories::TrajectorySample& contact_sample); // out : modified contact ref to give to tsid

        /** Correct the roll of the torso to have an equal left and right foot force measurment
         *  see Biped Walking Stabilization Based on Linear Inverted Pendulum Tracking
         *      by Shuuji Kajita, Mitsuharu Morisawa, Kanako Miura, Shin’ichiro Nakaoka,Kensuke Harada, Kenji Kaneko, Fumio Kanehiro and Kazuhito Yokoi
         *  see Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control
         *      by Stéphane Caron, Abderrahmane Kheddar, Olivier Tempier
        */
        void foot_force_difference_admittance(
            double dt, // controller dt
            double Mg, // Mass of the robot * gravity
            const Eigen::VectorXd& p_ffda, // 3d proportional gains
            double lf_normal_force, // normal left foot force from tsid solution (~= current model normal force)
            double rf_normal_force, // normal right foot force from tsid solution (~= current model normal force)
            const Eigen::Vector3d& lf_force, //measured left foot force vector
            const Eigen::Vector3d& rf_force, //measured right foot force vector
            const tsid::trajectories::TrajectorySample& torso_sample_ref, // tsid torso reference
            tsid::trajectories::TrajectorySample& torso_sample); //out : modified torso ref to give to tsid

        /** ZMP to feet forces and torques
         *  see Biped Walking Stabilization Based on Linear Inverted Pendulum Tracking
         *      by Shuuji Kajita, Mitsuharu Morisawa, Kanako Miura, Shin’ichiro Nakaoka,Kensuke Harada, Kenji Kaneko, Fumio Kanehiro and Kazuhito Yokoi
         * MODIFIED : I took a simpler heuristic for alpha
         * MODIFIED : Torque sign is different to adjust to Talos feet xy convention
        */
        double zmp_distributor(
            const double& robot_mass, // total mass of the robot
            const Eigen::Vector2d& zmp, // zmp
            const std::map<std::string, pinocchio::SE3>& contact_ref, // map of current contact position + rotation in the model
            const std::vector<std::string>& activated_contacts, // map of current activated contacts in the model
            Eigen::Matrix<double, 6, 1>& left_fref, // output left foot forces + torques command
            Eigen::Matrix<double, 6, 1>& right_fref); // output right foot forces + torques command

        /** compute zmp_distributor admittance control
         *  it makes a PD control between force_torque from zmp_distributor (measured zmp vs model zmp)
        */
        double zmp_distributor_admittance(
            double dt, // controller dt
            const Eigen::VectorXd& p, // proportional gains
            const Eigen::VectorXd& d, // derivative gains
            const double& robot_mass, // total mass of the robot
            const std::map<std::string, pinocchio::SE3>& contact_ref, // map of current contact position + rotation in the model
            const std::vector<std::string>& activated_contacts, // map of current activated contacts in the model
            const Eigen::Vector2d& zmp, // current measured zmp
            const tsid::trajectories::TrajectorySample& model_current_com, //pinocchio current com pos,vel,acc
            Eigen::Matrix<double, 6, 1>& left_fref, //output left foot forces + torques command
            Eigen::Matrix<double, 6, 1>& right_fref); //output right foot forces + torques command

        Eigen::Vector3d closest_point_on_line(
            const Eigen::Vector3d& point,
            const std::pair<Eigen::Vector3d, Eigen::Vector3d>& line);

        /***************** Not used for now ******************************/

        //Make the measured cop match the current desired cop thanks to imu data
        void com_imu_admittance(
            double dt, // controller dt
            const Eigen::VectorXd& d, // 6d derivative gains (for com velocity estimated with imu)
            const Eigen::MatrixXd& velocity, // imu velocity
            const tsid::trajectories::TrajectorySample& com_ref, // next com reference
            tsid::trajectories::TrajectorySample& se3_sample); //out : modified com ref to give to tsid

        /** compute zmp_distributor admittance control
         *  it makes a PD control between force_torque from zmp_distributor and measured force_torque
        */
        double direct_zmp_distributor_admittance(
            double dt, // controller dt
            const double& robot_mass,
            const Eigen::VectorXd& p, // proportional gains
            const Eigen::VectorXd& d, // derivative gains
            const tsid::trajectories::TrajectorySample& model_current_com, //pinocchio current com pos,vel,acc
            const std::map<std::string, pinocchio::SE3>& contact_ref, // map of current contact position + rotation in the model
            const std::vector<std::string>& activated_contacts, // map of current activated contacts in the model
            const Eigen::Matrix<double, 6, 1>& left_fref_sensor, //measured left foot forces + torques
            const Eigen::Matrix<double, 6, 1>& right_fref_sensor, //measured right foot forces + torques
            Eigen::Matrix<double, 6, 1>& left_fref, //output left foot forces + torques command
            Eigen::Matrix<double, 6, 1>& right_fref); //output right foot forces + torques command

    } // namespace stabilizer
} // namespace inria_wbc
#endif