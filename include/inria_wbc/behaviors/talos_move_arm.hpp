#ifndef IWBC_MOVE_ARM_HPP
#define IWBC_MOVE_ARM_HPP
#include <iostream>
#include <chrono>
#include <signal.h>

#include <inria_wbc/controllers/talos_pos_tracking.hpp>
#include <inria_wbc/utils/trajectory_handler.hpp>
#include <inria_wbc/behaviors/factory.hpp>

namespace inria_wbc
{
    namespace behaviors
    {
        class TalosMoveArm : public Behavior
        {
        public:
            TalosMoveArm(const inria_wbc::controllers::TalosBaseController::Params &params);
            TalosMoveArm() = delete;
            TalosMoveArm(const TalosMoveArm& other) = default;
            virtual std::shared_ptr<Behavior> clone() override { return std::make_shared<TalosMoveArm>(*this); }
            bool cmd(Eigen::VectorXd &) override;
            virtual ~TalosMoveArm() {}
        private:
            int time_ = 0;
            int traj_selector_ = 0;
            std::vector<std::vector<pinocchio::SE3>> trajectories_;
            std::vector<pinocchio::SE3> current_trajectory_;
        };
    } // namespace behaviors
} // namespace inria_wbc
#endif