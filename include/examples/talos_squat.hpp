#ifndef TALOS_SQUAT_HPP
#define TALOS_SQUAT_HPP
#include "examples/factory.hpp"
#include <iostream>
#include <chrono>
#include <signal.h>
#include "controllers/talos_pos_tracking.hpp"
#include "utils/trajectory_handler.hpp"

namespace tsid_sot
{
    namespace example
    {
        class TalosSquat : public Example
        {
        public:
            TalosSquat(const tsid_sot::controllers::TalosBaseController::Params &params);
            TalosSquat() = delete;
            Eigen::VectorXd cmd();

        private:
            int time_ = 0;
            int traj_selector_ = 0;
            std::vector<std::vector<Eigen::VectorXd>> trajectories_;
            std::vector<Eigen::VectorXd> current_trajectory_;
        };
    } // namespace example
} // namespace tsid_sot
#endif