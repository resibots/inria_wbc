#ifndef IWBC_TALOS_POS_TRACKER_HPP
#define IWBC_TALOS_POS_TRACKER_HPP

#include "inria_wbc/stabilizers/stabilizer_conf.hpp"
#include <boost/optional.hpp>
#include <inria_wbc/controllers/pos_tracker.hpp>
#include <inria_wbc/estimators/cop.hpp>
#include <inria_wbc/estimators/filtering.hpp>
#include <inria_wbc/safety/torque_collision_detection.hpp>
namespace inria_wbc {
    namespace controllers {
        class TalosPosTracker : public PosTracker {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            TalosPosTracker(const YAML::Node& config);
            TalosPosTracker(const TalosPosTracker& other) = delete;
            TalosPosTracker& operator=(const TalosPosTracker& o) const = delete;
            virtual ~TalosPosTracker(){};

            virtual void update(const SensorData& sensor_data = {}) override;

            virtual const Eigen::Vector3d& lf_force_filtered() const override { return _lf_force_filtered; }
            virtual const Eigen::Vector3d& rf_force_filtered() const override { return _rf_force_filtered; }

            const std::vector<std::string>& torque_sensor_joints() const { return _torque_collision_joints; }
            const safety::TorqueCollisionDetection& torque_collision_detector() const { return _torque_collision_detection; }
            bool collision_detected() const { return _collision_detected; }
            void clear_collision_detection();
            bool closed_loop() const { return _closed_loop; }
            void set_closed_loop(bool b) { _closed_loop = b; }

            void parse_stabilizer(const YAML::Node& config);
            void set_behavior_type(const std::string& bt);

            virtual const boost::optional<Eigen::Vector2d>& cop() const override { return _cop_estimator.cop(); }
            virtual const boost::optional<Eigen::Vector2d>& lcop() const override { return _cop_estimator.lcop_filtered(); }
            virtual const boost::optional<Eigen::Vector2d>& rcop() const override { return _cop_estimator.rcop_filtered(); }
            virtual const boost::optional<Eigen::Vector2d>& cop_raw() const override { return _cop_estimator.cop_raw(); }

        protected:
            virtual void parse_configuration(const YAML::Node& config);
            void parse_collision_thresholds(const std::string& config_path);
            void init_com();

            //sensor estimation & filtering
            estimators::Cop _cop_estimator;

            Eigen::Vector3d _lf_force_filtered;
            Eigen::Vector3d _rf_force_filtered;
            Eigen::Vector3d _lf_torque_filtered;
            Eigen::Vector3d _rf_torque_filtered;
            Eigen::Vector3d _imu_angular_vel_filtered;

            estimators::Filter::Ptr _lf_force_filter;
            estimators::Filter::Ptr _rf_force_filter;
            estimators::Filter::Ptr _lf_torque_filter;
            estimators::Filter::Ptr _rf_torque_filter;
            estimators::Filter::Ptr _imu_angular_vel_filter;

            //stabilisation parameters
            std::map<std::string, inria_wbc::stabilizer::StabConfig> _sconf_map;
            bool _use_stabilizer = false;
            bool _is_ss = false;

            //torque collision
            bool _use_torque_collision_detection;
            bool _collision_detected = false;
            safety::TorqueCollisionDetection _torque_collision_detection;
            estimators::Filter::Ptr _torque_collision_filter;
            std::vector<std::string> _torque_collision_joints;
            std::vector<int> _torque_collision_joints_ids;
            Eigen::VectorXd _torque_collision_threshold;
        };

    } // namespace controllers
} // namespace inria_wbc
#endif
