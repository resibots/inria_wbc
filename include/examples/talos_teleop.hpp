#ifndef TALOS_SQUAT_HPP
#define TALOS_SQUAT_HPP
#include "examples/factory.hpp"
#include <iostream>
#include <chrono>
#include <signal.h>
#include "controllers/talos_pos_tracking.hpp"
#include "utils/trajectory_handler.hpp"
#include "xsens_parser/xsens_joint_trajectory.hpp"

namespace tsid_sot
{
    namespace example
    {
        class TalosTeleop : public Example
        {
        public:
            TalosTeleop(const tsid_sot::controllers::TalosBaseController::Params &params);
            TalosTeleop() = delete;
            Eigen::VectorXd cmd();

        private:
            int time_ = 0;
            std::vector<std::vector<pinocchio::SE3>> trajectories_;
            std::shared_ptr<XSensJointTrajectory> xsens_trajectory_;
        };
    } // namespace example
} // namespace tsid_sot
#endif