#ifndef IWBC_SQUAT_HPP
#define IWBC_SQUAT_HPP
#include <chrono>
#include <iostream>
#include <signal.h>

#include <inria_wbc/behaviors/behavior.hpp>
#include <inria_wbc/controllers/talos_pos_tracking.hpp>
#include <inria_wbc/utils/trajectory_handler.hpp>

namespace inria_wbc {
    namespace behaviors {
        class TalosSquat : public Behavior {
        public:
            TalosSquat(const controller_ptr_t& controller);
            TalosSquat() = delete;
            TalosSquat(const TalosSquat& other) = delete;

            bool update() override;
            virtual ~TalosSquat() {}

        private:
            int time_ = 0;
            int traj_selector_ = 0;
            std::vector<std::vector<Eigen::VectorXd>> trajectories_;
            std::vector<Eigen::VectorXd> current_trajectory_;
            float trajectory_duration_ = 3; //will be changed if specified in yaml
            float motion_size_ = 0.2; //will be changed if specified in yaml
            bool print_cost_ = false;
        };
    } // namespace behaviors
} // namespace inria_wbc
#endif