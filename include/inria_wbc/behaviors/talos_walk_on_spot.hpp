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
            float dt_;
            int traj_selector_ = 0;
            //std::vector<std::vector<pinocchio::SE3>> trajectories_;
            std::vector<std::vector<Eigen::VectorXd>> trajectories_;
            std::vector<Eigen::VectorXd> current_trajectory_;
            float trajectory_duration_ = 3; //will be changed if specified in yaml
            float motion_size_ = 0.2;       //will be changed if specified in yaml

            // State machine stats for walking on the spot cycle
            int state_ = -1;
            enum States {
                MOVE_COM_LEFT,
                MOVE_COM_RIGHT,
                LIFT_UP_RF,
                LIFT_DOWN_RF,
                LIFT_UP_LF,
                LIFT_DOWN_LF,
            };
            bool first_run_ = true;
            int cycle_size_ = 6;
            int cycle_[6] {States::MOVE_COM_RIGHT, States::LIFT_UP_LF, States::LIFT_DOWN_LF, States::MOVE_COM_LEFT, States::LIFT_UP_RF, States::LIFT_DOWN_RF};
            
            // Left Foot
            pinocchio::SE3  lf_init_, lf_ref_;
            std::shared_ptr<tsid::trajectories::TrajectorySE3Constant> trajLf_;
            tsid::trajectories::TrajectorySample sampleLf_;

            // Right Foot
            pinocchio::SE3  rf_init_, rf_ref_;
            std::shared_ptr<tsid::trajectories::TrajectorySE3Constant> trajRf_;
            tsid::trajectories::TrajectorySample sampleRf_;

        };
    } // namespace behaviors
} // namespace inria_wbc
#endif
