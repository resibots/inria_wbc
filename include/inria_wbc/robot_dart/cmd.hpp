#ifndef IWBC_ROBORT_DART_CMD_HPP_
#define IWBC_ROBORT_DART_CMD_HPP_

#include <robot_dart/robot.hpp>
#include <robot_dart/robot_dart_simu.hpp>

namespace inria_wbc {
    namespace robot_dart {
        // compute torques from positions
        inline Eigen::VectorXd compute_spd(dart::dynamics::SkeletonPtr robot, const Eigen::VectorXd& targetpos, double dt)
        {
            Eigen::VectorXd q = robot->getPositions();
            Eigen::VectorXd dq = robot->getVelocities();

            float stiffness = 10000;
            float damping = 100;
            int ndofs = robot->getNumDofs();
            Eigen::MatrixXd Kp = Eigen::MatrixXd::Identity(ndofs, ndofs);
            Eigen::MatrixXd Kd = Eigen::MatrixXd::Identity(ndofs, ndofs);

            for (std::size_t i = 0; i < robot->getNumDofs(); ++i) {
                Kp(i, i) = stiffness;
                Kd(i, i) = damping;
            }
            for (std::size_t i = 0; i < 6; ++i) {
                Kp(i, i) = 0;
                Kd(i, i) = 0;
            }

            Eigen::MatrixXd invM = (robot->getMassMatrix() + Kd  *dt).inverse();
            Eigen::VectorXd p = -Kp * (q + dq * dt - targetpos);
            Eigen::VectorXd d = -Kd * dq;
            Eigen::VectorXd qddot = invM * (-robot->getCoriolisAndGravityForces() + p + d + robot->getConstraintForces());
            Eigen::VectorXd commands = p + d - Kd * qddot * dt;
            return commands;
        }

        // compute velocities from positions
        inline Eigen::VectorXd compute_velocities(dart::dynamics::SkeletonPtr robot, const Eigen::VectorXd& targetpos, double dt)
        {
            Eigen::VectorXd q = robot->getPositions();
            Eigen::VectorXd vel = (targetpos - q) / dt;
            return vel;
        }
    } // namespace robot_dart
} // namespace inria_wbc
#endif