#ifndef IWBC_TELEOP_HPP
#define IWBC_TELEOP_HPP
#include <iostream>
#include <chrono>
#include <signal.h>

#include <inria_wbc/controllers/talos_pos_tracking.hpp>
#include <inria_wbc/utils/trajectory_handler.hpp>
#include <inria_wbc/behaviors/factory.hpp>
#include "xsens_parser/xsens_joint_trajectory.hpp"

namespace inria_wbc
{
    namespace behaviors
    {
        class TalosTeleop : public Behavior
        {
        public:
            TalosTeleop(const inria_wbc::controllers::TalosBaseController::Params &params);
            TalosTeleop() = delete;
            TalosTeleop(const TalosTeleop &otehr) = default;
            virtual std::shared_ptr<Behavior> clone() override { return std::make_shared<TalosTeleop>(*this); }
            bool update() override;

        private:
            int time_ = 0;
            std::vector<std::vector<pinocchio::SE3>> trajectories_;
            std::shared_ptr<XSensJointTrajectory> xsens_trajectory_;
            pinocchio::SE3 lh_talos_init_, rh_talos_init_, lh_talos_cmd_, rh_talos_cmd_, lh_dhm_init_, rh_dhm_init_;
            pinocchio::SE3 floating_base_;
        };
    } // namespace behaviors
} // namespace inria_wbc
#endif