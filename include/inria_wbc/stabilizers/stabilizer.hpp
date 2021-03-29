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

        void com_admittance(
            double dt, //controller dt
            const Eigen::VectorXd& p, // proportional gains
            const Eigen::VectorXd& d, //derivative gains (for com velocity estimated with imu)
            const Eigen::MatrixXd& velocity, // imu velocity
            const Eigen::Vector2d& cop_filtered, //filtered cop estimation
            const tsid::trajectories::TrajectorySample& com_ref, //tsid com reference
            const tsid::InverseDynamicsFormulationAccForce::Data& data, //tsid data
            tsid::trajectories::TrajectorySample& se3_sample); //out : modified com ref to give to tsid

        void ankle_admittance(
            double dt, //controller dt
            const std::string& foot, //foot in contact with the ground "l" or "r" (left/right)
            const Eigen::Vector2d& cop_foot, //filtered foot cop estimation (from the foot force/torque sensor)
            const Eigen::VectorXd& p, // proportional gains
            pinocchio::SE3 ankle_ref, //tsid ankle reference of the same foot
            std::map<std::string, pinocchio::SE3> contact_ref, //tsid contact reference of the same foot
            tsid::trajectories::TrajectorySample& contact_sample, //out : modified contact ref to give to tsid
            tsid::trajectories::TrajectorySample& se3_sample); //out : modified ankle ref to give to tsid

        void foot_force_difference_admittance(
            double dt, //controller dt
            Eigen::VectorXd p_ffda, // proportional gains
            pinocchio::SE3 torso_ref, //tsid torso reference
            double lf_normal_force, //tsid normal left foot force
            double rf_normal_force, //tsid normal right foot force
            const Eigen::Vector3d& lf_force, //measured left foot force vector
            const Eigen::Vector3d& rf_force, //measured right foot force vector
            tsid::trajectories::TrajectorySample& torso_sample); //out : modified torso ref to give to tsid

        double zmp_distributor_admittance(
            double dt,
            const Eigen::VectorXd& p,
            const Eigen::VectorXd& d,
            const double& robot_mass,
            const Eigen::Vector2d& zmp,
            std::map<std::string, pinocchio::SE3> contact_ref,
            const std::vector<std::string>& activated_contacts,
            const tsid::InverseDynamicsFormulationAccForce::Data& data,
            const Eigen::Matrix<double, 3, Eigen::Dynamic>& left_foot_contact_points,
            const Eigen::Matrix<double, 3, Eigen::Dynamic>& right_foot_contact_points,
            Eigen::Matrix<double, 6, 1>& left_fref,
            Eigen::Matrix<double, 6, 1>& right_fref);

        double zmp_distributor(
            const double& robot_mass,
            const Eigen::Vector2d& zmp,
            std::map<std::string, pinocchio::SE3> contact_ref,
            const std::vector<std::string>& activated_contacts,
            const Eigen::Matrix<double, 3, Eigen::Dynamic>& left_foot_contact_points,
            const Eigen::Matrix<double, 3, Eigen::Dynamic>& right_foot_contact_points,
            Eigen::Matrix<double, 6, 1>& left_fref,
            Eigen::Matrix<double, 6, 1>& right_fref);

        Eigen::Vector3d closest_point_on_line(
            const Eigen::Vector3d point,
            const std::pair<Eigen::Vector3d, Eigen::Vector3d>& line);


    } // namespace stabilizer
} // namespace inria_wbc
#endif