#ifndef IWBC_TELEOP_HPP
#define IWBC_TELEOP_HPP
#include <iostream>
#include <chrono>
#include <signal.h>

#include <inria_wbc/controllers/talos_pos_tracking.hpp>
#include <inria_wbc/utils/trajectory_handler.hpp>
#include <inria_wbc/xsens_parser/xsens_joint_trajectory.hpp>

namespace inria_wbc
{
    namespace example
    {
        class TalosTeleop : public Example
        {
        public:
            TalosTeleop(const inria_wbc::controllers::TalosBaseController::Params &params);
            TalosTeleop() = delete;
            TalosTeleop(const TalosTeleop& c) { assert(0); /* not ready yet*/ }
            virtual std::shared_ptr<Behavior> clone() override { return std::make_shared<TalosTeleop>(*this); }
            bool cmd(Eigen::VectorXd &) override;

        private:
            int time_ = 0;
            std::vector<std::vector<pinocchio::SE3>> trajectories_;
            std::shared_ptr<XSensJointTrajectory> xsens_trajectory_;
        };
    } // namespace example
} // namespace inria_wbc
#endif