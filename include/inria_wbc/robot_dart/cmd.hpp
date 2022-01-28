#ifndef IWBC_ROBORT_DART_CMD_HPP_
#define IWBC_ROBORT_DART_CMD_HPP_

#include <robot_dart/robot.hpp>
#include <robot_dart/robot_dart_simu.hpp>

namespace inria_wbc {
    namespace robot_dart {
        // compute torques from positions
        inline Eigen::VectorXd compute_spd(const std::shared_ptr<::robot_dart::Robot>& robot, const Eigen::VectorXd& targetpos, double dt, const std::vector<std::string>& joints, bool floating_base = true)
        {
            Eigen::VectorXd q = robot->positions(joints);
            Eigen::VectorXd dq = robot->velocities(joints);

            float stiffness = 10000 / (dt * 1000); //tuned for 1000hz and then scaled for other freq
            float damping = 100 / (dt * 1000);
            int ndofs = joints.size();
            Eigen::MatrixXd Kp = Eigen::MatrixXd::Identity(ndofs, ndofs);
            Eigen::MatrixXd Kd = Eigen::MatrixXd::Identity(ndofs, ndofs);

            for (std::size_t i = 0; i < ndofs; ++i) {
                Kp(i, i) = stiffness;
                Kd(i, i) = damping;
            }

            if (robot->free() && floating_base) // floating base
                for (std::size_t i = 0; i < 6; ++i) {
                    Kp(i, i) = 0;
                    Kd(i, i) = 0;
                }

            Eigen::MatrixXd invM = (robot->mass_matrix(joints) + Kd * dt).inverse();
            Eigen::VectorXd p = -Kp * (q + dq * dt - targetpos);
            Eigen::VectorXd d = -Kd * dq;
            Eigen::VectorXd qddot = invM * (-robot->coriolis_gravity_forces(joints) + p + d + robot->constraint_forces(joints));
            Eigen::VectorXd commands = p + d - Kd * qddot * dt;
            return commands;
        }

        // compute velocities from positions
        inline Eigen::VectorXd compute_velocities(const std::shared_ptr<::robot_dart::Robot>& robot, const Eigen::VectorXd& targetpos, double dt, const std::vector<std::string>& joints)
        {
            Eigen::VectorXd q = robot->positions(joints);
            Eigen::VectorXd vel = (targetpos - q) / dt;
            return vel;
        }
    } // namespace robot_dart
} // namespace inria_wbc
#endif