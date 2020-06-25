#ifndef TALOS_SQUAT_HPP
#define TALOS_SQUAT_HPP
#include "examples/factory.hpp"
#include <iostream>
#include <chrono>
#include <signal.h>
#include "controllers/talos_pos_tracking.hpp"
#include "utils/trajectory_handler.hpp"

namespace tsid_sot
{
    namespace example
    {
        class TalosSquat : public Example
        {
        public:
            TalosSquat(const tsid_sot::controllers::TalosBaseController::Params &params,
                       const std::string &sot_config_path = "",
                       const std::string &fb_joint_name = "",
                       const std::vector<std::string> &mimic_joint_names = {},
                       bool verbose = false);
            TalosSquat() = delete;

            Eigen::VectorXd cmd();

            // std::shared_ptr<tsid_sot::controllers::TalosPosTracking> get_controller() { return controller_; };

        private:
            int time_ = 0;
            int traj_selector_ = 0;
            std::vector<std::vector<Eigen::VectorXd>> trajectories_;
            std::vector<Eigen::VectorXd> current_trajectory_;
            // std::shared_ptr<tsid_sot::controllers::TalosPosTracking> controller_;
        };
    } // namespace example
} // namespace tsid_sot
#endif