#ifndef IWBC_ROBORT_DART_UTILS_HPP_
#define IWBC_ROBORT_DART_UTILS_HPP_

#include <Eigen/Core>

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/math/Geometry.hpp"
#include <dart/dynamics/FreeJoint.hpp>

#include <robot_dart/robot.hpp>
#include <robot_dart/robot_dart_simu.hpp>

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

        //draw utils for model collisions
        inline std::vector<std::shared_ptr<::robot_dart::Robot>> create_spherical_members(
            const std::map<std::string, std::vector<std::pair<Eigen::Vector3d, float>>>& spherical_members,
            ::robot_dart::RobotDARTSimu& simu,
            const Eigen::Vector4d& color,
            const Eigen::Vector3d& translate = Eigen::Vector3d::Zero())
        {
            std::vector<std::shared_ptr<::robot_dart::Robot>> sphere_robots;
            for (auto& it : spherical_members) {
                auto sphere_vec = it.second;
                for (int i = 0; i < sphere_vec.size(); i++) {
                    auto pos = sphere_vec[i].first;
                    auto tf = Eigen::Isometry3d(Eigen::Translation3d(pos[0] + translate[0], pos[1] + translate[1], pos[2] + translate[2]));
                    auto diameter = sphere_vec[i].second;
                    auto sphere_robot = ::robot_dart::Robot::create_ellipsoid(Eigen::Vector3d(diameter, diameter, diameter), tf, "fixed", 1, color, "self-collision-" + std::to_string(i));
                    sphere_robot->skeleton()->getRootBodyNode()->getShapeNodes()[0]->getShape()->addDataVariance(dart::dynamics::Shape::DYNAMIC_PRIMITIVE);
                    sphere_robot->skeleton()->getRootBodyNode()->getShapeNodes()[0]->getShape()->addDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR);
                    sphere_robot->set_color_mode("aspect");
                    simu.add_visual_robot(sphere_robot);
                    sphere_robots.push_back(sphere_robot);
                }
            }
            return sphere_robots;
        }

        inline void update_spherical_members(
            const std::map<std::string, std::vector<std::pair<Eigen::Vector3d, float>>>& spherical_members,
            std::vector<std::shared_ptr<::robot_dart::Robot>>& sphere_robots,
            const Eigen::Vector4d& color,
            bool& is_colliding,
            const std::pair<std::pair<std::string, int>, std::pair<std::string, int>>& collision_index,
            const Eigen::Vector3d& translate = Eigen::Vector3d::Zero())
        {
            auto sphere_color = color;
            int j = 0;
            for (auto& it : spherical_members) {
                auto sphere_vec = it.second;
                for (int i = 0; i < sphere_vec.size(); i++) {
                    auto pos = sphere_vec[i].first;
                    auto tf = Eigen::Isometry3d(Eigen::Translation3d(pos[0] + translate[0], pos[1] + translate[1], pos[2] + translate[2]));

                    if (j >= sphere_robots.size())
                        std::cerr << "spherical_members size has changed because there is not enough sphere_robots" << j << " " << sphere_robots.size() << std::endl;

                    if (is_colliding) {
                        if ((collision_index.first.first == it.first && collision_index.first.second == i) || (collision_index.second.first == it.first && collision_index.second.second == i)) {
                            sphere_color = dart::Color::Red(1);
                        }
                        else {
                            sphere_color = dart::Color::Orange(0.2);
                        }
                    }
                    sphere_robots[j]->skeleton()->getRootBodyNode()->getParentJoint()->setTransformFromParentBodyNode(tf);
                    sphere_robots[j]->skeleton()->getRootBodyNode()->getShapeNodes()[0]->getVisualAspect()->setColor(sphere_color);
                    j++;
                }
            }
        }

    } // namespace robot_dart
} // namespace inria_wbc

#endif