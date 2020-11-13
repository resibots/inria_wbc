//
// Copyright (c) 2017 CNRS, NYU, MPI Tübingen
// Copyright (c) 2018 CNRS, NYU, MPI Tübingen
// This file is inspired by tsid, modified by Inria (2020)
// Copyright (c) 2020 Inria
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

#ifndef __invdyn_ex_task_hpp__
#define __invdyn_ex_task_hpp__

#include "tsid/math/constraint-equality.hpp"
#include "tsid/tasks/task-motion.hpp"
#include "tsid/trajectories/trajectory-base.hpp"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace tsid {
    namespace tasks {
        // this is a copy of SE3Equality
        class ExTask : public TaskMotion {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            typedef math::Index Index;
            typedef trajectories::TrajectorySample TrajectorySample;
            typedef math::Vector Vector;
            typedef math::ConstraintEquality ConstraintEquality;
            typedef pinocchio::Data Data;
            typedef pinocchio::Data::Matrix6x Matrix6x;
            typedef pinocchio::Motion Motion;
            typedef pinocchio::SE3 SE3;

            ExTask(const std::string& name,
                RobotWrapper& robot,
                const std::string& frameName);

            int dim() const;

            const ConstraintBase& compute(const double t,
                ConstRefVector q,
                ConstRefVector v,
                Data& data);

            const ConstraintBase& getConstraint() const;

            void setReference(TrajectorySample& ref);
            const TrajectorySample& getReference() const;
            const Vector& getDesiredAcceleration() const;
            Vector getAcceleration(ConstRefVector dv) const;
            virtual void setMask(math::ConstRefVector mask);
            const Vector& position_error() const;
            const Vector& velocity_error() const;

            const Vector& position() const;
            const Vector& velocity() const;
            const Vector& position_ref() const;
            const Vector& velocity_ref() const;

            const Vector& Kp() const;
            const Vector& Kd() const;
            void Kp(ConstRefVector Kp);
            void Kd(ConstRefVector Kp);

            Index frame_id() const;
            void useLocalFrame(bool local_frame);

        protected:
            std::string m_frame_name;
            Index m_frame_id;
            Motion m_p_error, m_v_error;
            Vector m_p_error_vec, m_v_error_vec;
            Vector m_p_error_masked_vec, m_v_error_masked_vec;
            Vector m_p, m_v;
            Vector m_p_ref, m_v_ref_vec;
            Motion m_v_ref, m_a_ref;
            SE3 m_M_ref, m_wMl;
            Vector m_Kp;
            Vector m_Kd;
            Vector m_a_des, m_a_des_masked;
            Motion m_drift;
            Vector m_drift_masked;
            Matrix6x m_J;
            Matrix6x m_J_rotated;
            ConstraintEquality m_constraint;
            TrajectorySample m_ref;
            bool m_local_frame;
        };

    } // namespace tasks
} // namespace tsid

#endif // ifndef  __invdyn_ex_task_hpp__
