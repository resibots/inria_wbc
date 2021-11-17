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

            virtual const Eigen::Vector2d& cop() const override
            {
                if (_cop_estimator.cop())
                    return _cop_estimator.cop().value();
                else
                    return cst::V2_1000;
            }

            virtual const Eigen::Vector2d& lcop() const override
            {
                if (_cop_estimator.lcop_filtered())
                    return _cop_estimator.lcop_filtered().value();
                else
                    return cst::V2_1000;
            }

            virtual const Eigen::Vector2d& rcop() const override
            {
                if (_cop_estimator.rcop_filtered())
                    return _cop_estimator.rcop_filtered().value();
                else
                    return cst::V2_1000;
            }

            virtual const Eigen::Vector2d& cop_raw() const override
            {
                if (_cop_estimator.cop_raw())
                    return _cop_estimator.cop_raw().value();
                else
                    return cst::V2_1000;
            }
            virtual const Eigen::Vector3d& lf_force_filtered() const override { return _lf_force_filtered; }
            virtual const Eigen::Vector3d& rf_force_filtered() const override { return _rf_force_filtered; }

            bool closed_loop() const { return _closed_loop; }
            void set_closed_loop(bool b) { _closed_loop = b; }

        protected:
            void init_com();
            void parse_stabilizer(const YAML::Node& config) override;

            //sensor estimation & filtering
            estimators::Cop _cop_estimator;

            Eigen::Vector3d _lf_force_filtered;
            Eigen::Vector3d _rf_force_filtered;
            Eigen::Vector3d _lf_torque_filtered;
            Eigen::Vector3d _rf_torque_filtered;

            estimators::Filter::Ptr _lf_force_filter;
            estimators::Filter::Ptr _rf_force_filter;
            estimators::Filter::Ptr _lf_torque_filter;
            estimators::Filter::Ptr _rf_torque_filter;

            //stabilisation parameters
            bool _use_stabilizer = true;
            double _torso_max_roll = 0.25;

            Eigen::VectorXd _com_gains;
            Eigen::VectorXd _ankle_gains;
            Eigen::VectorXd _ffda_gains;

            bool _activate_zmp = false;
            Eigen::VectorXd _zmp_p;
            Eigen::VectorXd _zmp_d;
            Eigen::VectorXd _zmp_w;
        };

    } // namespace controllers
} // namespace inria_wbc
#endif
