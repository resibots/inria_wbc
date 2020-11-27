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

#include "tsid/tasks/task-self-collision.hpp"
#include "tsid/robots/robot-wrapper.hpp"

namespace tsid {
    namespace tasks {
        using namespace math;
        using namespace trajectories;
        using namespace pinocchio;

        TaskSelfCollision::TaskSelfCollision(const std::string& name,
            RobotWrapper& robot,
            const std::string& frameName) : TaskMotion(name, robot),
                                            m_frame_name(frameName),
                                            m_constraint(name, 1, robot.nv())
        {
            assert(m_robot.model().existFrame(frameName));
            m_frame_id = m_robot.model().getFrameId(frameName);
            m_Kp = 0.;
            m_Kd = 0.;
            m_grad_C.setZero(3);
            m_J.setZero(6, robot.nv());
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
            return m_frame_id;
        }

        void TaskSelfCollision::setP0(const Vector3& p0)
        {
            m_p0 = p0;
        }

        const Vector3& TaskSelfCollision::getP0() const
        {
            return m_p0;
        }

        void TaskSelfCollision::setCoef(const double coef)
        {
            m_coef = coef;
        }

        const double TaskSelfCollision::getCoef() const
        {
            return m_coef;
        }

        void TaskSelfCollision::setr0(const double r0)
        {
            m_r0 = r0;
        }

        const double TaskSelfCollision::getr0() const
        {
            return m_r0;
        }
        /*
     void TaskSelfCollision::compute_C(Vector3 pos){
      Vector3 diff = pos-m_p0;
      double square_norm = diff.dot(diff);
      assert(square_norm > 0.);
      m_C(0,0) = m_coef/square_norm;
    } 

    void TaskSelfCollision::compute_grad_C(Vector3 pos){
      Vector3 diff = pos-m_p0;
      double square_norm = diff.dot(diff);
      assert(square_norm > 0.);

      m_grad_C = -2*m_coef/(square_norm*square_norm) * diff;
    }
 
    void TaskSelfCollision::compute_Hessian_C(Vector3 pos){
      Vector3 diff = pos-m_p0;
      double square_norm = diff.dot(diff);
      assert(square_norm > 0.);
      m_Hessian_C = 8*m_coef/(square_norm*square_norm*square_norm) * diff * diff.transpose() - 2 / (square_norm*square_norm) * (Eigen::Matrix<double, 3, 3>::Identity());
    } 
    */

        void TaskSelfCollision::compute_C(const Vector3& pos)
        {
            Vector3 diff = pos - m_p0;
            double square_norm = diff.dot(diff);
            double norm = sqrt(square_norm);
            if (norm > m_r0) {
                m_C(0, 0) = 0;
            }
            else {
                assert(square_norm > 0.);
                m_C(0, 0) = 0.5 * m_coef * (1 / norm - 1 / m_r0) * (1 / norm - 1 / m_r0);
            }
        }

        void TaskSelfCollision::compute_grad_C(const Vector3& pos)
        {
            Vector3 diff = pos - m_p0;
            double square_norm = diff.dot(diff);
            double norm = sqrt(square_norm);
            if (norm > m_r0) {
                m_grad_C = Vector3::Zero();
            }
            else {
                assert(square_norm > 0.);
                m_grad_C = -m_coef * (1 / (square_norm * square_norm) - 1 / (m_r0 * square_norm * norm)) * diff;
            }
        }

        void TaskSelfCollision::compute_Hessian_C(const Vector3& pos)
        {
            Vector3 diff = pos - m_p0;
            double square_norm = diff.dot(diff);
            double norm = sqrt(square_norm);
            if (norm > m_r0) {
                m_Hessian_C = Eigen::Matrix<double, 3, 3>::Zero();
            }
            else {
                assert(square_norm > 0.);
                m_Hessian_C = m_coef * ((4 / (square_norm * square_norm * square_norm) - 3 / (m_r0 * square_norm * square_norm * norm)) * diff * diff.transpose() - (1 / (square_norm * square_norm) - 1 / (m_r0 * square_norm * norm)) * (Eigen::Matrix<double, 3, 3>::Identity()));
            }
        }

        const ConstraintBase& TaskSelfCollision::compute(const double,
            ConstRefVector,
            ConstRefVector v,
            Data& data)
        {
            //// Get pos (position of the frame)
            SE3 oMi;
            Motion v_frame;
            Motion a_frame;
            m_robot.framePosition(data, m_frame_id, oMi);
            m_robot.frameVelocity(data, m_frame_id, v_frame);
            m_robot.frameClassicAcceleration(data, m_frame_id, a_frame);
            auto pos = oMi.translation();
            m_drift = a_frame.linear();

            //// Get Frame Position jacobian (TODO: Local Jacobian VS World Jacobian)
            m_robot.frameJacobianLocal(data, m_frame_id, m_J);
            // get the pos jacobian (3 x nactuated)
            auto J = m_J.block(0, 0, 3, m_robot.nv());

            //// Compute C(pos)
            compute_C(pos);
            //// Compute gradient of C(pos)
            compute_grad_C(pos);
            //// Compute Hessian of C(pos)
            compute_Hessian_C(pos);

#ifdef LOG_REACHING_LOW
            std::cout << "p0: " << m_p0 << std::endl;
            std::cout << "r0: " << m_r0 << std::endl;
            std::cout << "pos: " << pos << std::endl;

            std::cout << "vel: " << v_frame.linear() << std::endl;
            std::cout << "acc: " << a_frame.linear() << std::endl;
            std::cout << "dx=J*dq: " << J * v << std::endl;
            std::cout << "dq: " << v.transpose() << std::endl;
            std::cout << "J: " << J << std::endl;
            std::cout << "drift" << m_drift << std::endl;

            std::cout << "C: " << m_C << std::endl;
            std::cout << "grad_C: " << m_grad_C << std::endl;
            std::cout << "Hessian C: " << m_Hessian_C << std::endl;

            std::cout << "A: " << m_grad_C.transpose() * J << std::endl;
            std::cout << "b: " << (-((m_Hessian_C * J * v).transpose() * J * v + m_grad_C.transpose() * (m_drift + m_Kd * J * v) + m_Kp * m_C)) << std::endl;
#endif

            // Set A
            m_constraint.setMatrix(m_grad_C.transpose() * J);
            // Set b (note: m_drift = dJ(q)*dq)
            m_constraint.setVector(-((m_Hessian_C * J * v).transpose() * J * v + m_grad_C.transpose() * (m_drift + m_Kd * J * v) + m_Kp * m_C));
            return m_constraint;
        }

    } // namespace tasks
} // namespace tsid
