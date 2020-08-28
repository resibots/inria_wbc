#include "inria_wbc/behaviors/talos_teleop.hpp"

namespace inria_wbc
{
    namespace behaviors
    {
        static AutoRegister<TalosTeleop> __talos_squat("talos-teleop");

        TalosTeleop::TalosTeleop(const inria_wbc::controllers::TalosBaseController::Params &params) :
            Behavior(std::make_shared<inria_wbc::controllers::TalosPosTracking>(params))
        {
            //////////////////// INIT STACK OF TASK //////////////////////////////////////
            controller_ = std::make_shared<inria_wbc::controllers::TalosPosTracking>(params);
            //////////////////// INIT XSensTraj //////////////////////////////////////
            std::string teleoperation_file = "/home/user/xsens_parser/tests/etc/covid19-002.mvnx";
            int start_frame = 0;
            int end_frame = 250;
            YAML::Node config = YAML::LoadFile(controller_->params().sot_config_path);
            inria_wbc::utils::parse(teleoperation_file, "teleoperation_file", config, false, "EXAMPLE");
            inria_wbc::utils::parse(start_frame, "start_frame", config, false, "EXAMPLE");
            inria_wbc::utils::parse(end_frame, "end_frame", config, false, "EXAMPLE");
            xsens_trajectory_ = std::make_shared<XSensJointTrajectory>(teleoperation_file, start_frame, end_frame);
            xsens_trajectory_->initialize(controller_->dt());

            //////////////////// Go To initial xsens position  //////////////////////////////////////
            auto lh_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_se3_ref("lh");
            auto lh_final = lh_init;
            lh_final.translation() = xsens_trajectory_->getDhmCurrentFramePin("left_hand").translation();
            auto rh_init = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_se3_ref("rh");
            auto rh_final = rh_init;
            rh_final.translation() = xsens_trajectory_->getDhmCurrentFramePin("left_hand").translation();
            float trajectory_duration = 2;
            trajectories_.push_back(trajectory_handler::compute_traj(lh_init, lh_final, params.dt, trajectory_duration));
            trajectories_.push_back(trajectory_handler::compute_traj(rh_init, rh_final, params.dt, trajectory_duration));
            std::cout << lh_init.translation().transpose() << " " << lh_final.translation().transpose() << std::endl;
            std::cout << rh_init.translation().transpose() << " " << rh_final.translation().transpose() << std::endl;
        }

        bool TalosTeleop::cmd(Eigen::VectorXd &q)
        {
            if (time_ < trajectories_[0].size())
            {
                std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_se3_ref(trajectories_[0][time_], "lh");
                std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_se3_ref(trajectories_[1][time_], "rh");
            }
            else
            {
                std::cout << "INITIALIZATION IS DONE" << std::endl;
            }
            // if (xsens_trajectory_->updateStep())
            // {
            //     auto ref = xsens_trajectory_->getDhmCurrentFramePin("left_hand");
            //     std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_se3_ref(ref, "lh");
            //     ref = xsens_trajectory_->getDhmCurrentFramePin("right_hand");
            //     std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_se3_ref(ref, "rh");
            // }
            if (controller_->solve())
            {
                time_++;
                q.resize(controller_->q(false).size()); //size 50)
                q = controller_->q(false);
                std::cout << q.size() << std::endl;
                return true;
            }
            else
            {
                return false;
            }
        }

    } // namespace behaviors
} // namespace inria_wbc
