#ifndef IWBC_SWITCH_FOOT
#define IWBC_SWITCH_FOOT
#include <chrono>
#include <iostream>
#include <signal.h>

#include <inria_wbc/behaviors/factory.hpp>
#include <inria_wbc/controllers/talos_pos_tracking.hpp>
#include <inria_wbc/utils/trajectory_handler.hpp>

namespace inria_wbc {
    namespace behaviors {

        class WalkOnSpot : public Behavior {
        public:
            WalkOnSpot(const inria_wbc::controllers::TalosBaseController::Params& params);
            WalkOnSpot() = delete;
            WalkOnSpot(const WalkOnSpot& otehr) = default;
            bool update() override;

        private:
            int time_ = 0;
            float dt_;
            int traj_selector_ = 0;
            std::vector<std::vector<Eigen::VectorXd>> trajectories_;
            std::vector<Eigen::VectorXd> current_com_trajectory_;
            std::vector<pinocchio::SE3> current_foot_trajectory_;
            float trajectory_duration_ = 3; //will be changed if specified in yaml
            float motion_size_ = 0.2; //will be changed if specified in yaml

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
            std::vector<States> cycle_;
        };
    } // namespace behaviors
} // namespace inria_wbc
#endif
