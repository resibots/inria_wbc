
#ifndef IWBC_ESTIMATOR_COP_HPP
#define IWBC_ESTIMATOR_COP_HPP

#include <Eigen/Core>
#include <algorithm>
#include <boost/optional.hpp>
#include <inria_wbc/estimators/filtering.hpp>
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
                _cop_filtered = boost::none;
                _lcop_filtered = boost::none;
                _rcop_filtered = boost::none;
                _cop_raw = boost::none;
                _lcop_raw = boost::none;
                _rcop_raw = boost::none;
                _cop_filter = std::make_shared<estimators::MovingAverageFilter>(2, history_size); //cop data of size 2
                _lcop_filter = std::make_shared<estimators::MovingAverageFilter>(2, history_size); //cop data of size 2
                _rcop_filter = std::make_shared<estimators::MovingAverageFilter>(2, history_size); //cop data of size 2
            }

            // returns the filtered CoP
            std::vector<boost::optional<Eigen::Vector2d>> update(
                const Eigen::Vector2d& ref,
                const Eigen::Vector3d& lf_pos, const Eigen::Vector3d& rf_pos,
                const Eigen::Vector3d& lf_torque, const Eigen::Vector3d& lf_force,
                const Eigen::Vector3d& rf_torque, const Eigen::Vector3d& rf_force, bool memory = true);

            void set_history_size(size_t h)
            {
                _history_size = h;
                _cop_filter->set_window_size(h);
                _lcop_filter->set_window_size(h);
                _rcop_filter->set_window_size(h);
            }
            size_t history_size() const { return _history_size; }
            void set_sample_time(size_t t) { _sample_time = t; }
            // estimates of cop
            const boost::optional<Eigen::Vector2d>& cop() const { return cop_filtered(); }
            const boost::optional<Eigen::Vector2d>& cop_filtered() const { return _cop_filtered; }
            const boost::optional<Eigen::Vector2d>& lcop_filtered() const { return _lcop_filtered; }
            const boost::optional<Eigen::Vector2d>& rcop_filtered() const { return _rcop_filtered; }
            const boost::optional<Eigen::Vector2d>& cop_raw() const { return _cop_raw; }
            const boost::optional<Eigen::Vector2d>& lcop_raw() const { return _lcop_raw; }
            const boost::optional<Eigen::Vector2d>& rcop_raw() const { return _rcop_raw; }

            void check_nan(boost::optional<Eigen::Vector2d>& cop);
            constexpr float fmin() { return FMIN; }

        protected:
            double _sample_time;
            size_t _history_size;

            boost::optional<Eigen::Vector2d> _cop_raw, _lcop_raw, _rcop_raw; // last computed CoP
            boost::optional<Eigen::Vector2d> _cop_filtered, _lcop_filtered, _rcop_filtered; // filtered CoP
            estimators::Filter::Ptr _cop_filter, _lcop_filter, _rcop_filter; // filters

            Eigen::Vector2d _compute_cop(const Eigen::Vector3d& lf_pos, const Eigen::Vector3d& rf_pos,
                const Eigen::Vector2d& lcop_raw, const Eigen::Vector2d& rcop_raw,
                const Eigen::Vector3d& lf_force, const Eigen::Vector3d& rf_force);

            Eigen::Vector2d _compute_foot_cop(const Eigen::Vector3d& foot_pos,
                const Eigen::Vector3d& torque, const Eigen::Vector3d& force);
        };
    } // namespace estimators
} // namespace inria_wbc

#endif
