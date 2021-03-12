#ifndef IWBC_FRANKA_POS_TRACKER_HPP
#define IWBC_FRANKA_POS_TRACKER_HPP

#include <inria_wbc/controllers/pos_tracker.hpp>
#include <inria_wbc/estimators/cop.hpp>

namespace inria_wbc {
    namespace controllers {

        class FrankaPosTracker : public PosTracker {
        public:
            FrankaPosTracker(const Params& params);
            FrankaPosTracker(const FrankaPosTracker& other) = delete;
            FrankaPosTracker& operator=(const FrankaPosTracker& o) const = delete;
            virtual ~FrankaPosTracker(){};


            virtual void update(const SensorData& sensor_data = {}) override;

        };

    } // namespace controllers
} // namespace inria_wbc
#endif
