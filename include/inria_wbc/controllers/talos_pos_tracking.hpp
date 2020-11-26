#ifndef IWBC_TALOS_POS_TRACKER_HPP
#define IWBC_TALOS_POS_TRACKER_HPP

#include <inria_wbc/controllers/pos_tracker.hpp>
#include <inria_wbc/estimators/cop.hpp>

namespace inria_wbc {
    namespace controllers {

        class TalosPosTracking : public PosTracker {
        public:
            TalosPosTracking(const Params& params);
            TalosPosTracking(const TalosPosTracking& other) = delete;
            TalosPosTracking& operator=(const TalosPosTracking& o) const = delete;
            virtual ~TalosPosTracking(){};

            virtual void update(const SensorData& sensor_data) override;
            virtual const Eigen::Vector2d& cop() const override { return _cop_estimator.cop(); }

        protected:
            virtual void parse_configuration_yaml(const std::string& sot_config_path);
            estimators::Cop _cop_estimator;
            bool _use_stabilizer = true;
            Eigen::Vector2d _stabilizer_p = Eigen::Vector2d(0.005, 0.005);
            Eigen::Vector2d _stabilizer_d = Eigen::Vector2d(0, 0);
        };

    } // namespace controllers
} // namespace inria_wbc
#endif
