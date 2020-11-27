//
// Copyright (c) 2017 CNRS
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

#ifndef __invdyn_task_self_collision_hpp__
#define __invdyn_task_self_collision_hpp__

#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/fwd.hpp"
#include "tsid/tasks/task-motion.hpp"
#include "tsid/trajectories/trajectory-base.hpp"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace tsid {
    namespace tasks {

        class TaskSelfCollision : public TaskMotion {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            typedef math::Index Index;
            typedef math::Vector Vector;
            typedef math::Vector3 Vector3;
            typedef math::ConstraintEquality ConstraintEquality;
            typedef pinocchio::Data::Matrix6x Matrix6x;

            TaskSelfCollision(const std::string& name,
                RobotWrapper& robot,
                const std::string& frameName);
            virtual TaskSelfCollision() {}
            
            int dim() const;

            const ConstraintBase& compute(const double t,
                ConstRefVector q,
                ConstRefVector v,
                Data& data);

            const ConstraintBase& getConstraint() const;

            void setP0(const Vector3& p0);
            const Vector3& getP0() const;
            void setCoef(const double coef);
            const double getCoef() const;
            void setr0(const double r0);
            const double getr0() const;

            double Kp() const { return m_Kp; }
            double Kd() const { return m_Kd; }

            void Kp(const double Kp);
            void Kd(const double Kp);
            Index frame_id() const;

            void compute_C(const Vector3& x);
            void compute_grad_C(const Vector3& x);
            void compute_Hessian_C(const Vector3& x);

        protected:
            std::string m_frame_name;
            Index m_frame_id;

            double m_Kp;
            double m_Kd;
            Vector3 m_drift;
            Eigen::Matrix<double, 1, 1> m_C;
            Vector3 m_grad_C;
            Eigen::Matrix<double, 3, 3> m_Hessian_C;
            Matrix6x m_J;
            Vector3 m_p0;
            double m_r0;
            double m_coef;
            ConstraintEquality m_constraint;
        };

    } // namespace tasks
} // namespace tsid

#endif // ifndef __invdyn_task_pos_avoidance_hpp__
