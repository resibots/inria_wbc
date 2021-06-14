#ifndef IWBC_EX_CONTROLLER_HPP
#define IWBC_EX_CONTROLLER_HPP

#include <inria_wbc/controllers/pos_tracker.hpp>

namespace inria_wbc {
    namespace controllers {

        class ExController : public PosTracker {
        public:
            ExController(const YAML::Node& config);
            ExController(const ExController& other) = delete;
            ExController& operator=(const ExController& o) const = delete;
            virtual ~ExController(){};

            virtual void update(const SensorData& sensor_data = {}) override;

            bool closed_loop() const { return _closed_loop; }
            void set_closed_loop(bool b) { _closed_loop = b; }

        protected:
            virtual void parse_configuration(const YAML::Node& config);
            bool _closed_loop = false;
        };

    } // namespace controllers
} // namespace inria_wbc
#endif