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
#include <unordered_map>

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
                const std::string& frameName,
                const std::unordered_map<std::string, double>& frames,
                double radius,
                double p);
            virtual ~TaskSelfCollision() {}

            int dim() const;

            const ConstraintBase& compute(const double t,
                ConstRefVector q,
                ConstRefVector v,
                Data& data);

            const ConstraintBase& getConstraint() const;

            double Kp() const { return m_Kp; }
            double Kd() const { return m_Kd; }

            void Kp(const double kp) { m_Kp = kp; }
            void Kd(const double kd) { m_Kd = kd; }
            Index frame_id() const;

            const std::vector<Vector3>& avoided_frames_positions() const { return m_avoided_frames_positions; }
            const std::vector<double>& avoided_frames_r0s() const { return m_avoided_frames_r0s; }
            bool collision(int i) const { return m_collisions[i]; }
            double radius() const { return m_radius; }
        protected:
            bool compute_C(const Vector3& x, const std::vector<Vector3>& frames_positions);
            void compute_grad_C(const Vector3& x, const std::vector<Vector3>& frames_positions);
            void compute_Hessian_C(const Vector3& x, const std::vector<Vector3>& frames_positions);

            std::string m_tracked_frame_name; // name of the body that we track
            Index m_tracked_frame_id; // id of the body that we track (from the model)
            std::unordered_map<std::string, double> m_avoided_frames_names; // names of the bodies to avoid and radius
            std::vector<Index> m_avoided_frames_ids; // id of the bodies to avoid
            std::vector<double> m_avoided_frames_r0s; // radius for each avoided frame

            double m_Kp;
            double m_Kd;

            Eigen::Matrix<double, 1, 1> m_C;
            Vector3 m_grad_C;
            Eigen::Matrix<double, 3, 3> m_Hessian_C;

            double m_p;
            double m_radius;
            ConstraintEquality m_constraint;

            Vector3 m_drift;
            Matrix6x m_J; // jacobian of the tracked frame
            std::vector<Matrix6x> m_Js; // jacobian of the other frames

            std::vector<Vector3> m_avoided_frames_positions;

            std::vector<bool> m_collisions;

            Eigen::MatrixXd m_A;
            Eigen::VectorXd m_B;
        };

    } // namespace tasks
} // namespace tsid

#endif // ifndef __invdyn_task_pos_avoidance_hpp__
