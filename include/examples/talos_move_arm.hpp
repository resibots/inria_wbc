#ifndef TALOS_MOVE_ARM_HPP
#define TALOS_MOVE_ARM_HPP
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
        class TalosMoveArm : public Example
        {
        public:
            TalosMoveArm(const tsid_sot::controllers::TalosBaseController::Params &params);
            TalosMoveArm() = delete;
            Eigen::VectorXd cmd();

        private:
            int time_ = 0;
            int traj_selector_ = 0;
            std::vector<std::vector<pinocchio::SE3>> trajectories_;
            std::vector<pinocchio::SE3> current_trajectory_;
        };
    } // namespace example
} // namespace tsid_sot
#endif