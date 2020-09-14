#ifndef IWBC_SQUAT_HPP
#define IWBC_SQUAT_HPP
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
        class TalosSquat : public Behavior
        {
        public:
            TalosSquat(const inria_wbc::controllers::TalosBaseController::Params &params);
            TalosSquat() = delete;
            TalosSquat(const TalosSquat &other) = delete;

            bool update() override;
            virtual ~TalosSquat() {}

        private:
            int time_ = 0;
            int traj_selector_ = 0;
            std::vector<std::vector<Eigen::VectorXd>> trajectories_;
            std::vector<Eigen::VectorXd> current_trajectory_;
            float trajectory_duration_ = 3; //will be changed if specified in yaml
            float motion_size_ = 0.2;       //will be changed if specified in yaml
        };
    } // namespace behaviors
} // namespace inria_wbc
#endif