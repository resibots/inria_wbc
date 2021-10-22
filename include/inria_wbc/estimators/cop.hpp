
#ifndef IWBC_ESTIMATOR_COP_HPP
#define IWBC_ESTIMATOR_COP_HPP

#include <Eigen/Core>
#include <algorithm>
#include <deque>
#include <vector>

namespace inria_wbc {
    namespace estimators {
        // computes the CoP with the FT sensors + a filter
         static constexpr float FMIN = 30;
         
        class Cop {
        public:
            Cop(double sample_time = 0.001, size_t history_size = 5)
                : _sample_time(sample_time),
                  _history_size(history_size)
            {
                _cop_filtered.setZero();
                _lcop_filtered.setZero();
                _rcop_filtered.setZero();
                _cop_raw.setZero();
                _lcop_raw.setZero();
                _rcop_raw.setZero();
            }

            // returns the filtered CoP
            bool update(
                const Eigen::Vector2d& ref,
                const Eigen::Vector3d& lf_pos, const Eigen::Vector3d& rf_pos,
                const Eigen::Vector3d& lf_torque, const Eigen::Vector3d& lf_force,
                const Eigen::Vector3d& rf_torque, const Eigen::Vector3d& rf_force);
            void set_history_size(size_t h) { _history_size = h; }
            size_t history_size() const { return _history_size; }
            void set_sample_time(size_t t) { _sample_time = t; }
            // estimates of cop
            const Eigen::Vector2d& cop() const { return cop_filtered(); }
            const Eigen::Vector2d& cop_filtered() const { return _cop_filtered; }
            const Eigen::Vector2d& lcop_filtered() const { return _lcop_filtered; }
            const Eigen::Vector2d& rcop_filtered() const { return _rcop_filtered; }
            const Eigen::Vector2d& cop_raw() const { return _cop_raw; }
            const Eigen::Vector2d& lcop_raw() const { return _lcop_raw; }
            const Eigen::Vector2d& rcop_raw() const { return _rcop_raw; }
            const float& fmin() { return FMIN; }

        protected:
            double _sample_time;
            size_t _history_size;

            Eigen::Vector2d _cop_raw, _lcop_raw, _rcop_raw; // last computed CoP
            Eigen::Vector2d _cop_filtered, _lcop_filtered, _rcop_filtered; // filtered CoP
            std::deque<Eigen::Vector2d> _cop_buffer, _lcop_buffer, _rcop_buffer; // previous values of cop

            std::vector<Eigen::Vector2d> _compute_cop(
                const Eigen::Vector3d& lf_pos, const Eigen::Vector3d& rf_pos,
                const Eigen::Vector3d& lf_torque, const Eigen::Vector3d& lf_force,
                const Eigen::Vector3d& rf_torque, const Eigen::Vector3d& rf_force);
        };
    } // namespace estimators
} // namespace inria_wbc

#endif
