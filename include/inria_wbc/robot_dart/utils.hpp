#ifndef IWBC_ROBORT_DART_UTILS_HPP_
#define IWBC_ROBORT_DART_UTILS_HPP_

#include <Eigen/Core>

#include <dart/dynamics/FreeJoint.hpp>

namespace inria_wbc {
    namespace robot_dart {
        // Dart represents the floating base position using a 6D spatial vector
        // whereas tsid / inria_wbc uses 3D pos + quaternion (7D)
        inline Eigen::VectorXd floating_base_pos(const Eigen::VectorXd& q_dart)
        {
            Eigen::Isometry3d p = dart::dynamics::FreeJoint::convertToTransform(q_dart.head(6));
            Eigen::Quaterniond quat = Eigen::Quaterniond(p.rotation());
            Eigen::VectorXd q_tsid(3 + 4); // make space for the quaternion
            q_tsid << p.translation(), quat.coeffs();
            return q_tsid;
        }

        // Dart represents the floating base velocity using a 6D spatial vector (rotation + position)
        // whereas tsid / inria_wbc uses 3D pos + 3D rotation
        inline Eigen::VectorXd floating_base_vel(const Eigen::VectorXd& qdot_dart)
        {
            // we just need to swap
            Eigen::VectorXd qdot_tsid(3 + 3);
            qdot_tsid << qdot_dart.head(6).tail(3), qdot_dart.head(3);
            return qdot_tsid;
        }

        inline Eigen::VectorXd filter_cmd(const Eigen::VectorXd& cmds,
            const std::vector<std::string>& orig_joints,
            const std::vector<std::string>& new_joints)
        {
            Eigen::VectorXd new_commands = Eigen::VectorXd::Zero(new_joints.size());
            for (size_t n = 0; n < new_joints.size(); ++n)
                for (size_t k = 0; k < orig_joints.size(); ++k)
                    if (new_joints[n] == orig_joints[k])
                        new_commands(n) = cmds(k);
            return new_commands;
        }

    } // namespace robot_dart
} // namespace inria_wbc

#endif