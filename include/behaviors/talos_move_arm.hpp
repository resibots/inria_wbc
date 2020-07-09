#ifndef TALOS_MOVE_ARM_HPP
#define TALOS_MOVE_ARM_HPP
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
        class TalosMoveArm : public Behavior
        {
        public:
            TalosMoveArm(const tsid_sot::controllers::TalosBaseController::Params &params);
            TalosMoveArm() = delete;
            Eigen::VectorXd cmd();
            virtual ~TalosMoveArm() {}
        private:
            int time_ = 0;
            int traj_selector_ = 0;
            std::vector<std::vector<pinocchio::SE3>> trajectories_;
            std::vector<pinocchio::SE3> current_trajectory_;
        };
    } // namespace behaviors
} // namespace tsid_sot
#endif