#ifndef IWBC_EX_CONTROLLER_HPP
#define IWBC_EX_CONTROLLER_HPP

#include <inria_wbc/controllers/pos_tracker.hpp>

namespace inria_wbc {
    namespace controllers {

        class ExController : public PosTracker {
        public:
            ExController(const Params& params);
            ExController(const ExController& other) = delete;
            ExController& operator=(const ExController& o) const = delete;
            virtual ~ExController(){};

            virtual void update(const SensorData& sensor_data = {}) override;

        protected:
            virtual void parse_configuration_yaml(const std::string& sot_config_path);
        };

    } // namespace controllers
} // namespace inria_wbc
 #endif