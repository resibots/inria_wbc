//
// Copyright (c) 2017-2020 CNRS, Inria
//
// This file is part of tsid
// tsid is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// tsid is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// tsid If not, see
// <http://www.gnu.org/licenses/>.
//

//#define DEBUG_POS_AVOIDANCE
#include <cmath>

#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/tasks/task-self-collision.hpp"

namespace tsid {
    namespace tasks {
        using namespace math;
        using namespace trajectories;
        using namespace pinocchio;

        TaskSelfCollision::TaskSelfCollision(const std::string& name,
            RobotWrapper& robot,
            const std::string& tracked_frame_name,
            const std::unordered_map<std::string, double>& avoided_frames_names,
            double radius,
            double p)
            : TaskMotion(name, robot),
              m_tracked_frame_name(tracked_frame_name),
              m_avoided_frames_names(avoided_frames_names),
              m_radius(radius),
              m_p(p),
              m_constraint(name, 1, robot.nv()),
              m_Js(avoided_frames_names.size()),
              m_avoided_frames_positions(avoided_frames_names.size())

        {
            assert(m_robot.model().existFrame(m_tracked_frame_name));
            m_tracked_frame_id = m_robot.model().getFrameId(m_tracked_frame_name);
            m_Kp = 0.;
            m_Kd = 0.;
            m_grad_C.setZero(3);
            m_J.setZero(6, robot.nv());
            for (size_t i = 0; i < m_Js.size(); ++i)
                m_Js[i].setZero(6, robot.nv());

            for (const auto& it : m_avoided_frames_names) {
                assert(m_robot.model().existFrame(it.first));
                m_avoided_frames_ids.push_back(m_robot.model().getFrameId(it.first));
                m_avoided_frames_r0s.push_back(it.second);
            }
            m_collisions.resize(m_avoided_frames_ids.size());
        }

        int TaskSelfCollision::dim() const
        {
            return 1;
        }

        const ConstraintBase& TaskSelfCollision::getConstraint() const
        {
            return m_constraint;
        }

        Index TaskSelfCollision::frame_id() const
        {
            return m_tracked_frame_id;
        }

        const ConstraintBase& TaskSelfCollision::compute(const double,
            ConstRefVector q,
            ConstRefVector v,
            Data& data)
        {
            // pos & Jacobian of the tracked frame
            SE3 oMi;
            Motion v_frame;
            Motion a_frame;
            m_robot.framePosition(data, m_tracked_frame_id, oMi);
            //a_frame = m_robot.frameAccelerationWorldOriented(data, m_tracked_frame_id);
            m_robot.frameClassicAcceleration(data, m_tracked_frame_id, a_frame);
            //a_frame = data.a[m_tracked_frame_id];

            auto pos = oMi.translation();
            m_drift = a_frame.linear();
            m_robot.frameJacobianWorld(data, m_tracked_frame_id, m_J);
            //m_robot.frameJacobianLocal(data, m_tracked_frame_id, m_J);

            auto J1 = m_J.block(0, 0, 3, m_robot.nv());

            m_A = Eigen::MatrixXd::Zero(1, m_robot.nv());
            m_B = Eigen::MatrixXd::Zero(1, 1);
            std::fill(m_collisions.begin(), m_collisions.end(), false);

            for (size_t i = 0; i < m_avoided_frames_ids.size(); ++i) {
                // pos & Jacobian
                m_robot.framePosition(data, m_avoided_frames_ids[i], oMi);
                m_avoided_frames_positions[i] = oMi.translation();

                // distance with tracked frame
                const Vector3& pos2 = m_avoided_frames_positions[i];
                Vector3 diff = pos - pos2;
                double square_norm = diff.dot(diff);
                double norm = sqrt(square_norm);
                double r = m_avoided_frames_r0s[i];

                double eps = 1e-9; // we consider that we influence if above eps
                double p = m_p;
                double a = (r + m_radius); // * pow(-log(eps), -1.0 / p); //about 0.5;

                // if in the influence zone
                if (norm <= a) {
                    m_collisions[i] = true;
                }
                static const Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
                //if (norm < r + m_radius) // why do we need this??
                {
                    // std::cout << "activated for:" << m_tracked_frame_name << " norm:" << norm << " r:" << r << " radius:" << m_radius << std::endl;
                    m_robot.frameJacobianWorld(data, m_avoided_frames_ids[i], m_Js[i]);
                    //a_frame = m_robot.frameAccelerationWorldOriented(data, m_avoided_frames_ids[i]);
                    m_robot.frameClassicAcceleration(data, m_avoided_frames_ids[i], a_frame); // TODO dJ.dq in world frame
                    //a_frame = data.a[m_avoided_frames_ids[i]];
                    auto J = J1 - m_Js[i].block(0, 0, 3, m_robot.nv());
                    Eigen::Vector3d drift = m_drift - a_frame.linear();

                    m_C(0, 0) = exp(-pow(norm / a, p));
                    m_grad_C = -(p / pow(a, p) * pow(norm, p - 2)) * exp(-pow(norm / a, p)) * diff;
                    m_Hessian_C = exp(-pow(norm / a, p)) * ((p * p / pow(a, 2 * p) * pow(norm, 2 * p - 4) - p * (p - 2) / pow(a, p) * pow(norm, p - 4)) * diff * diff.transpose() - (p / pow(a, p) * pow(norm, p - 2)) * I);

                    // A
                    m_A += m_grad_C.transpose() * J;
                    // B (note: m_drift = dJ(q)*dq)
                    m_B += -((m_Hessian_C * J * v).transpose() * J * v + m_grad_C.transpose() * (-drift + m_Kd * J * v) + m_Kp * m_C);
                } // else 0 for everything
            }

            m_constraint.setMatrix(m_A);
            m_constraint.setVector(m_B);
            return m_constraint;
        }

    } // namespace tasks
} // namespace tsid
