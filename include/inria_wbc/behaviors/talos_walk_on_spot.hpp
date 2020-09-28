#ifndef IWBC_SWITCH_FOOT
#define IWBC_SWITCH_FOOT
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

        class WalkOnSpot : public Behavior
        {
        public:
            WalkOnSpot(const inria_wbc::controllers::TalosBaseController::Params &params);
            WalkOnSpot() = delete;
            WalkOnSpot(const WalkOnSpot &otehr) = default;
            //virtual std::shared_ptr<Behavior> clone() override { return std::make_shared<TestWbcCOM>(*this); }
            bool update() override;

        private:
            int time_ = 0;
            int traj_selector_ = 0;
            //std::vector<std::vector<pinocchio::SE3>> trajectories_;
            std::vector<std::vector<Eigen::VectorXd>> trajectories_;
            std::vector<Eigen::VectorXd> current_trajectory_;
            float trajectory_duration_ = 3; //will be changed if specified in yaml
            float motion_size_ = 0.2;       //will be changed if specified in yaml
            
        };
    } // namespace behaviors
} // namespace inria_wbc
#endif
