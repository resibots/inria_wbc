//
// Copyright (c) 2017-2020 CNRS, NYU, MPI Tübingen, Inria
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

#include "tsid/math/utils.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/tasks/ex_task.hpp"

namespace tsid {
    namespace tasks {
        using namespace std;
        using namespace math;
        using namespace trajectories;
        using namespace pinocchio;

        ExTask::ExTask(const std::string& name,
            RobotWrapper& robot,
            const std::string& frameName) : TaskMotion(name, robot),
                                            m_frame_name(frameName),
                                            m_constraint(name, 6, robot.nv()),
                                            m_ref(12, 6)
        {
            assert(m_robot.model().existFrame(frameName));
            m_frame_id = m_robot.model().getFrameId(frameName);

            m_v_ref.setZero();
            m_a_ref.setZero();
            m_M_ref.setIdentity();
            m_wMl.setIdentity();
            m_p_error_vec.setZero(6);
            m_v_error_vec.setZero(6);
            m_p.resize(12);
            m_v.resize(6);
            m_p_ref.resize(12);
            m_v_ref_vec.resize(6);
            m_Kp.setZero(6);
            m_Kd.setZero(6);
            m_a_des.setZero(6);
            m_J.setZero(6, robot.nv());
            m_J_rotated.setZero(6, robot.nv());

            m_mask.resize(6);
            m_mask.fill(1.);
            setMask(m_mask);

            m_local_frame = true;
        }

        void ExTask::setMask(math::ConstRefVector mask)
        {
            TaskMotion::setMask(mask);
            int n = dim();
            m_constraint.resize(n, (unsigned int)m_J.cols());
            m_p_error_masked_vec.resize(n);
            m_v_error_masked_vec.resize(n);
            m_drift_masked.resize(n);
            m_a_des_masked.resize(n);
        }

        int ExTask::dim() const
        {
            return (int)m_mask.sum();
        }

        const Vector& ExTask::Kp() const { return m_Kp; }

        const Vector& ExTask::Kd() const { return m_Kd; }

        void ExTask::Kp(ConstRefVector Kp)
        {
            assert(Kp.size() == 6);
            m_Kp = Kp;
        }

        void ExTask::Kd(ConstRefVector Kd)
        {
            assert(Kd.size() == 6);
            m_Kd = Kd;
        }

        void ExTask::setReference(TrajectorySample& ref)
        {
            m_ref = ref;
            
            assert(ref.getValue().size() == 12);
            m_M_ref.translation(ref.getValue().head<3>());
            m_M_ref.rotation(MapMatrix3(&ref.getValue()(3), 3, 3));
            
            m_v_ref = Motion(ref.getDerivative());
            m_a_ref = Motion(ref.getSecondDerivative());
        }

        void ExTask::setReference(const SE3& ref)
        {
            TrajectorySample s(12, 6);
            Eigen::VectorXd ref_vec(12);
            tsid::math::SE3ToVector(ref, ref_vec);
            s.setValue(ref_vec);
            setReference(s);
        }

        const TrajectorySample& ExTask::getReference() const
        {
            return m_ref;
        }

        const Vector& ExTask::position_error() const
        {
            return m_p_error_masked_vec;
        }

        const Vector& ExTask::velocity_error() const
        {
            return m_v_error_masked_vec;
        }

        const Vector& ExTask::position() const
        {
            return m_p;
        }

        const Vector& ExTask::velocity() const
        {
            return m_v;
        }

        const Vector& ExTask::position_ref() const
        {
            return m_p_ref;
        }

        const Vector& ExTask::velocity_ref() const
        {
            return m_v_ref_vec;
        }

        const Vector& ExTask::getDesiredAcceleration() const
        {
            return m_a_des_masked;
        }

        Vector ExTask::getAcceleration(ConstRefVector dv) const
        {
            return m_constraint.matrix() * dv + m_drift_masked;
        }

        Index ExTask::frame_id() const
        {
            return m_frame_id;
        }

        const ConstraintBase& ExTask::getConstraint() const
        {
            return m_constraint;
        }

        void ExTask::useLocalFrame(bool local_frame)
        {
            m_local_frame = local_frame;
        }

        const ConstraintBase& ExTask::compute(const double,
            ConstRefVector,
            ConstRefVector,
            Data& data)
        {
            SE3 oMi;
            Motion v_frame;
            m_robot.framePosition(data, m_frame_id, oMi);
            m_robot.frameVelocity(data, m_frame_id, v_frame);
            m_robot.frameClassicAcceleration(data, m_frame_id, m_drift);

            // @todo Since Jacobian computation is cheaper in world frame
            // we could do all computations in world frame
            m_robot.frameJacobianLocal(data, m_frame_id, m_J);

            errorInSE3(oMi, m_M_ref, m_p_error); // pos err in local frame
            SE3ToVector(m_M_ref, m_p_ref);
            SE3ToVector(oMi, m_p);

            // Transformation from local to world
            m_wMl.rotation(oMi.rotation());

            if (m_local_frame) {
                m_p_error_vec = m_p_error.toVector();
                m_v_error = m_wMl.actInv(m_v_ref) - v_frame; // vel err in local frame

                // desired acc in local frame
                m_a_des = m_Kp.cwiseProduct(m_p_error_vec)
                    + m_Kd.cwiseProduct(m_v_error.toVector())
                    + m_wMl.actInv(m_a_ref).toVector();
            }
            else {
                m_p_error_vec = m_wMl.toActionMatrix() * // pos err in local world-oriented frame
                    m_p_error.toVector();

                // cout<<"m_p_error_vec="<<m_p_error_vec.head<3>().transpose()<<endl;
                // cout<<"oMi-m_M_ref  ="<<-(oMi.translation()-m_M_ref.translation()).transpose()<<endl;

                m_v_error = m_v_ref - m_wMl.act(v_frame); // vel err in local world-oriented frame

                m_drift = m_wMl.act(m_drift);

                // desired acc in local world-oriented frame
                m_a_des = m_Kp.cwiseProduct(m_p_error_vec)
                    + m_Kd.cwiseProduct(m_v_error.toVector())
                    + m_a_ref.toVector();

                // Use an explicit temporary `m_J_rotated` here to avoid allocations.
                m_J_rotated.noalias() = m_wMl.toActionMatrix() * m_J;
                m_J = m_J_rotated;
            }

            m_v_error_vec = m_v_error.toVector();
            m_v_ref_vec = m_v_ref.toVector();
            m_v = v_frame.toVector();

            int idx = 0;
            for (int i = 0; i < 6; i++) {
                if (m_mask(i) != 1.)
                    continue;

                m_constraint.matrix().row(idx) = m_J.row(i);
                m_constraint.vector().row(idx) = (m_a_des - m_drift.toVector()).row(i);
                m_a_des_masked(idx) = m_a_des(i);
                m_drift_masked(idx) = m_drift.toVector()(i);
                m_p_error_masked_vec(idx) = m_p_error_vec(i);
                m_v_error_masked_vec(idx) = m_v_error_vec(i);

                idx += 1;
            }

            return m_constraint;
        }
    } // namespace tasks
} // namespace tsid
