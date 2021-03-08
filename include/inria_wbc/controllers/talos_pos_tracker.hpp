#ifndef IWBC_TALOS_POS_TRACKER_HPP
#define IWBC_TALOS_POS_TRACKER_HPP

#include <inria_wbc/controllers/pos_tracker.hpp>
#include <inria_wbc/estimators/cop.hpp>
#include <inria_wbc/estimators/filtering.hpp>
#include <inria_wbc/safety/torque_collision_detection.hpp>

namespace inria_wbc {
    namespace controllers {

        class TalosPosTracker : public PosTracker {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            TalosPosTracker(const Params& params);
            TalosPosTracker(const TalosPosTracker& other) = delete;
            TalosPosTracker& operator=(const TalosPosTracker& o) const = delete;
            virtual ~TalosPosTracker(){};

            virtual void update(const SensorData& sensor_data = {}) override;
            virtual const Eigen::Vector2d& cop() const override { return _cop_estimator.cop(); }
            virtual const Eigen::Vector2d& lcop() const override { return _cop_estimator.lcop_filtered(); }
            virtual const Eigen::Vector2d& rcop() const override { return _cop_estimator.rcop_filtered(); }
            virtual const Eigen::Vector2d& cop_raw() const { return _cop_estimator.cop_raw(); }
            virtual const Eigen::Vector2d& lcop_raw() const override { return _cop_estimator.lcop_raw(); }
            virtual const Eigen::Vector2d& rcop_raw() const override { return _cop_estimator.rcop_raw(); }

            const std::vector<std::string>& torque_sensor_joints() const { return _torque_collision_joints; }
            const safety::TorqueCollisionDetection& torque_collision_detector() const { return _torque_collision_detection; }
            bool collision_detected() const { return _collision_detected; }
            void clear_collision_detection();

            void com_admittance(
                const Eigen::VectorXd& p,
                const Eigen::VectorXd& d,
                const Eigen::MatrixXd& velocity,
                const Eigen::Vector2d& cop_filtered,
                const tsid::trajectories::TrajectorySample& com_ref,
                tsid::InverseDynamicsFormulationAccForce::Data data,
                tsid::trajectories::TrajectorySample& se3_sample);

            void ankle_admittance(
                double dt,
                const std::string& foot,
                const Eigen::Vector2d& cop_foot,
                const Eigen::VectorXd& p,
                pinocchio::SE3 ankle_ref,
                std::map<std::string, pinocchio::SE3> contact_ref,
                tsid::trajectories::TrajectorySample& contact_sample,
                tsid::trajectories::TrajectorySample& se3_sample);

            void foot_force_difference_admittance(
                double dt,
                float torso_max_roll,
                Eigen::VectorXd p_ffda,
                pinocchio::SE3 torso_ref,
                double lf_normal_force,
                double rf_normal_force,
                const Eigen::Vector3d& lf_force,
                const Eigen::Vector3d& rf_force,
                tsid::trajectories::TrajectorySample& torso_sample,
                std::unordered_map<std::string, tsid::math::Vector> ac_forces);

        protected:
            virtual void parse_configuration_yaml(const std::string& sot_config_path);
            void parse_collision_thresholds(const std::string& config_path);

            estimators::Cop _cop_estimator;
            bool _use_stabilizer = true;
            double _torso_max_roll = 0.3;

            Eigen::VectorXd _stabilizer_p;
            Eigen::VectorXd _stabilizer_d;
            Eigen::VectorXd _stabilizer_p_ankle;
            Eigen::VectorXd _stabilizer_p_ffda;

            std::map<std::string, pinocchio::SE3> _contact_ref;

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
