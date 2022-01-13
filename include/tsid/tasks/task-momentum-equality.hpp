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

#ifndef __invdyn_task_momentum_equality_hpp__
#define __invdyn_task_momentum_equality_hpp__

#include "tsid/math/fwd.hpp"
#include "tsid/tasks/task-motion.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace tsid
{
  namespace tasks
  {

    class TaskMEquality : public TaskMotion
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef math::Index Index;
      typedef trajectories::TrajectorySample TrajectorySample;
      typedef math::Vector Vector;
      typedef math::Vector3 Vector3;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef pinocchio::Data::Matrix6x Matrix6x;


      TaskMEquality(const std::string & name, 
                      RobotWrapper & robot);

      void setMask(math::ConstRefVector mask);
      int dim() const;

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     Data & data);

      const ConstraintBase & getConstraint() const;

      void setReference(const TrajectorySample & ref);
      const TrajectorySample & getReference() const;

      const Vector & getDesiredMomentumDerivative() const;
      Vector getdMomentum(ConstRefVector dv) const;

      const Vector & momentum_error() const;
      const Vector & momentum() const;
      const Vector & momentum_masked() const;
      const Vector & momentum_ref() const;
      const Vector & dmomentum_ref() const;

      const Vector & Kp();
      const Vector & Kd();
      void Kp(ConstRefVector Kp);
      void Kd(ConstRefVector Kp);

    protected:
      Vector m_Kp;
      Vector m_Kd;
      Vector m_L_error, m_dL_error;
      Vector m_dL_des;
      
      Vector m_drift;
      Vector m_L, m_dL;
      
      Vector m_L_error_masked;
      Vector m_L_masked;
      Vector m_dL_des_masked;
      Vector m_drift_masked;
      
      TrajectorySample m_ref;
      ConstraintEquality m_constraint;
    };
    
  }
}

#endif // ifndef __invdyn_task_am_equality_hpp__
