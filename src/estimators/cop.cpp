#include <cmath>
#include <inria_wbc/estimators/cop.hpp>
#include <iostream>
#include <numeric>

namespace inria_wbc {
    namespace estimators {

        bool Cop::update(
            const Eigen::Vector2d& ref,
            const Eigen::Vector3d& lf_pos, const Eigen::Vector3d& rf_pos,
            const Eigen::Vector3d& lf_torque, const Eigen::Vector3d& lf_force,
            const Eigen::Vector3d& rf_torque, const Eigen::Vector3d& rf_force)
        {
            // if we are in the air, returns false
            if (lf_force.norm() < fmin() && rf_force.norm() < fmin())
                return false;
            auto cop_data = _compute_cop(lf_pos, rf_pos, lf_torque, lf_force, rf_torque, rf_force);
            _lcop_raw = cop_data[0];
            _rcop_raw = cop_data[1];
            _cop_raw = cop_data[2];

            // store in _cop_buffer for filtering
            _cop_buffer.push_back(_cop_raw);
            _lcop_buffer.push_back(_lcop_raw);
            _rcop_buffer.push_back(_rcop_raw);
            if (_cop_buffer.size() > _history_size) {
                _cop_buffer.pop_front();
                _lcop_buffer.pop_front();
                _rcop_buffer.pop_front();
            }

            // moving average filtering : average the values in history
            _cop_filtered
                = std::accumulate(_cop_buffer.begin(), _cop_buffer.end(), (Eigen::Vector2d)Eigen::Vector2d::Zero()) / _cop_buffer.size();
            _lcop_filtered
                = std::accumulate(_lcop_buffer.begin(), _lcop_buffer.end(), (Eigen::Vector2d)Eigen::Vector2d::Zero()) / _lcop_buffer.size();
            _rcop_filtered
                = std::accumulate(_rcop_buffer.begin(), _rcop_buffer.end(), (Eigen::Vector2d)Eigen::Vector2d::Zero()) / _rcop_buffer.size();

            if (_cop_buffer.size() < _history_size)
                return false;
            return true;
        }

        // Intro to humanoid robotics (Kajita et al.), p80, 3.26
        // see also: https://github.com/stack-of-tasks/sot-dynamic-pinocchio/blob/master/src/zmp-from-forces.cpp

        std::vector<Eigen::Vector2d> Cop::_compute_cop(const Eigen::Vector3d& lf_pos, const Eigen::Vector3d& rf_pos,
            const Eigen::Vector3d& lf_torque, const Eigen::Vector3d& lf_force,
            const Eigen::Vector3d& rf_torque, const Eigen::Vector3d& rf_force)
        {
            // CoP left foot
            Eigen::Vector2d lcop_raw = lf_pos.head(2);
            lcop_raw(0) += (-lf_torque(1) - lf_pos(2) * lf_force(0)) / lf_force(2);
            lcop_raw(1) += (lf_torque(0) - lf_pos(2) * lf_force(1)) / lf_force(2);

            // CoP right foot
            Eigen::Vector2d rcop_raw = rf_pos.head(2);
            rcop_raw(0) += (-rf_torque(1) - rf_pos(2) * rf_force(0)) / rf_force(2);
            rcop_raw(1) += (rf_torque(0) - rf_pos(2) * rf_force(1)) / rf_force(2);

            double Fz_ratio_l = lf_force(2) / (lf_force(2) + rf_force(2));
            double Fz_ratio_r = rf_force(2) / (lf_force(2) + rf_force(2));

            Eigen::Vector2d cop_in_lft_raw = Fz_ratio_l * lcop_raw + Fz_ratio_r * (rcop_raw + rf_pos.head(2) - lf_pos.head(2));
            Eigen::Vector2d cop_in_rft_raw = Fz_ratio_l * (lcop_raw + lf_pos.head(2) - rf_pos.head(2)) + Fz_ratio_r * rcop_raw;
            Eigen::Vector2d cop = cop_in_lft_raw * Fz_ratio_l + cop_in_rft_raw * Fz_ratio_r;

            std::vector<Eigen::Vector2d> cop_data{lcop_raw, rcop_raw, cop};
            return cop_data;
        }
    } // namespace estimators
} // namespace inria_wbc