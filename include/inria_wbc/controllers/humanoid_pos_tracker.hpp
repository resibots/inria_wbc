#ifndef IWBC_HUMANOID_POS_TRACKER_HPP
#define IWBC_HUMANOID_POS_TRACKER_HPP

#include <inria_wbc/controllers/pos_tracker.hpp>
#include <inria_wbc/estimators/cop.hpp>
#include <inria_wbc/estimators/filtering.hpp>
#include <inria_wbc/safety/torque_collision_detection.hpp>

namespace inria_wbc {
    namespace controllers {
        namespace cst {
            static const Eigen::Vector2d V2_1000 = Eigen::Vector2d::Constant(1000);
        }
        // this is a pos-tracker with stabilization
        class HumanoidPosTracker : public PosTracker {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            HumanoidPosTracker(const YAML::Node& config);
            HumanoidPosTracker(const HumanoidPosTracker& other) = delete;
            HumanoidPosTracker& operator=(const HumanoidPosTracker& o) const = delete;
            virtual ~HumanoidPosTracker(){};

            virtual void update(const SensorData& sensor_data = {}) override;

            virtual const boost::optional<Eigen::Vector2d>& cop() const override { return _cop_estimator.cop(); }
            virtual const boost::optional<Eigen::Vector2d>& lcop() const override { return _cop_estimator.lcop_filtered(); }
            virtual const boost::optional<Eigen::Vector2d>& rcop() const override { return _cop_estimator.rcop_filtered(); }
            virtual const boost::optional<Eigen::Vector2d>& cop_raw() const override { return _cop_estimator.cop_raw(); }

            virtual const Eigen::Vector3d& lf_force_filtered() const override { return _lf_force_filtered; }
            virtual const Eigen::Vector3d& rf_force_filtered() const override { return _rf_force_filtered; }

            bool closed_loop() const { return _closed_loop; }
            void set_closed_loop(bool b) { _closed_loop = b; }

            virtual void set_behavior_type(const std::string& bt) override;

        protected:

            void init_com();
            virtual void parse_stabilizer(const YAML::Node& config);

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
            std::map<std::string, inria_wbc::stabilizer::StabConfig> _stabilizer_configs;
            bool _use_stabilizer = false;
            bool _is_ss = false;
        };

    } // namespace controllers
} // namespace inria_wbc
#endif
