#include <cmath>
#include <inria_wbc/estimators/cop.hpp>
#include <iostream>
#include <numeric>

namespace inria_wbc {
    namespace estimators {

        std::vector<bool> Cop::update(
            const Eigen::Vector2d& ref,
            const Eigen::Vector3d& lf_pos, const Eigen::Vector3d& rf_pos,
            const Eigen::Vector3d& lf_torque, const Eigen::Vector3d& lf_force,
            const Eigen::Vector3d& rf_torque, const Eigen::Vector3d& rf_force)
        {

            if (lf_force.norm() > fmin()) {
                _lcop_raw = _compute_foot_cop(lf_pos, lf_torque, lf_force);
                _lcop_filtered = _lcop_filter->filter(_lcop_raw);
            }
            else {
                _lcop_raw.setZero();
                _lcop_filtered.setZero();
                _lcop_filter->reset();
            }

            if (rf_force.norm() > fmin()) {
                _rcop_raw = _compute_foot_cop(rf_pos, rf_torque, rf_force);
                _rcop_filtered = _rcop_filter->filter(_rcop_raw);
            }
            else {
                _rcop_raw.setZero();
                _rcop_filtered.setZero();
                _rcop_filter->reset();
            }

            if (lf_force.norm() > fmin() && rf_force.norm() > fmin()) {
                _cop_raw = _compute_cop(lf_pos, rf_pos, _lcop_raw, _rcop_raw, lf_torque, lf_force, rf_torque, rf_force);
                _cop_filtered = _cop_filter->filter(_cop_raw);
            }
            else {
                _cop_raw.setZero();
                _cop_filtered.setZero();
                _cop_filter->reset();
            }

            return {_cop_filter->data_ready(), _lcop_filter->data_ready(), _rcop_filter->data_ready()};
        }

        // Intro to humanoid robotics (Kajita et al.), p80, 3.26
        // see also: https://github.com/stack-of-tasks/sot-dynamic-pinocchio/blob/master/src/zmp-from-forces.cpp
        Eigen::Vector2d Cop::_compute_cop(const Eigen::Vector3d& lf_pos, const Eigen::Vector3d& rf_pos,
            const Eigen::Vector2d& lcop_raw, const Eigen::Vector2d& rcop_raw,
            const Eigen::Vector3d& lf_torque, const Eigen::Vector3d& lf_force,
            const Eigen::Vector3d& rf_torque, const Eigen::Vector3d& rf_force)
        {

            double Fz_ratio_l = lf_force(2) / (lf_force(2) + rf_force(2));
            double Fz_ratio_r = rf_force(2) / (lf_force(2) + rf_force(2));

            Eigen::Vector2d cop_in_lft_raw = Fz_ratio_l * lcop_raw + Fz_ratio_r * (rcop_raw + rf_pos.head(2) - lf_pos.head(2));
            Eigen::Vector2d cop_in_rft_raw = Fz_ratio_l * (lcop_raw + lf_pos.head(2) - rf_pos.head(2)) + Fz_ratio_r * rcop_raw;
            Eigen::Vector2d cop = cop_in_lft_raw * Fz_ratio_l + cop_in_rft_raw * Fz_ratio_r;

            return cop;
        }

        // Intro to humanoid robotics (Kajita et al.), p80, 3.26
        // see also: https://github.com/stack-of-tasks/sot-dynamic-pinocchio/blob/master/src/zmp-from-forces.cpp
        Eigen::Vector2d Cop::_compute_foot_cop(const Eigen::Vector3d& foot_pos,
            const Eigen::Vector3d& torque, const Eigen::Vector3d& force)
        {
            // CoP of one foot
            Eigen::Vector2d cop_raw = foot_pos.head(2);
            cop_raw(0) += (-torque(1) - foot_pos(2) * force(0)) / force(2);
            cop_raw(1) += (torque(0) - foot_pos(2) * force(1)) / force(2);

            return cop_raw;
        }
    } // namespace estimators
} // namespace inria_wbc