#ifndef TALOS_SQUAT_HPP
#define TALOS_SQUAT_HPP
#include <iostream>
#include <chrono>
#include <signal.h>
#include "controllers/talos_pos_tracking.hpp"
#include "utils/trajectory_handler.hpp"
#include "behaviors/factory.hpp"

namespace tsid_sot
{
    namespace behaviors
    {
        class TalosSquat : public Behavior
        {
        public:
            TalosSquat(const tsid_sot::controllers::TalosBaseController::Params &params);
            TalosSquat() = delete;
            Eigen::VectorXd cmd();
            virtual ~TalosSquat() {}
        private:
            int time_ = 0;
            int traj_selector_ = 0;
            std::vector<std::vector<Eigen::VectorXd>> trajectories_;
            std::vector<Eigen::VectorXd> current_trajectory_;
        };
    } // namespace behaviors
} // namespace tsid_sot
#endif