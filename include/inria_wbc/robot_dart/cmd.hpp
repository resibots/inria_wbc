#ifndef IWBC_ROBORT_DART_CMD_HPP_
#define IWBC_ROBORT_DART_CMD_HPP_

#include <robot_dart/robot.hpp>
#include <robot_dart/robot_dart_simu.hpp>

namespace inria_wbc {
    namespace robot_dart {
        // compute torques from positions
        inline Eigen::VectorXd compute_spd(const std::shared_ptr<::robot_dart::Robot>&  robot, const Eigen::VectorXd& targetpos, double dt)
        {
            Eigen::VectorXd q = robot->positions();
            Eigen::VectorXd dq = robot->velocities();

            float stiffness = 10000;
            float damping = 100;
            int ndofs = robot->num_dofs();
            Eigen::MatrixXd Kp = Eigen::MatrixXd::Identity(ndofs, ndofs);
            Eigen::MatrixXd Kd = Eigen::MatrixXd::Identity(ndofs, ndofs);

            for (std::size_t i = 0; i < robot->num_dofs(); ++i) {
                Kp(i, i) = stiffness;
                Kd(i, i) = damping;
            }

            if (robot->free()) // floating base
                for (std::size_t i = 0; i < 6; ++i) {
                    Kp(i, i) = 0;
                    Kd(i, i) = 0;
                }

            Eigen::MatrixXd invM = (robot->mass_matrix() + Kd  * dt).inverse();
            Eigen::VectorXd p = -Kp * (q + dq * dt - targetpos);
            Eigen::VectorXd d = -Kd * dq;
            Eigen::VectorXd qddot = invM * (-robot->coriolis_gravity_forces() + p + d + robot->skeleton()->getConstraintForces());
            Eigen::VectorXd commands = p + d - Kd * qddot * dt;
            return commands;
        }

        // compute velocities from positions
        inline Eigen::VectorXd compute_velocities(const std::shared_ptr<::robot_dart::Robot>&  robot, const Eigen::VectorXd& targetpos, double dt)
        {
            Eigen::VectorXd q = robot->positions();
            Eigen::VectorXd vel = (targetpos - q) / dt;
            return vel;
        }
    } // namespace robot_dart
} // namespace inria_wbc
#endif