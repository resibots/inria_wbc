#include "inria_wbc/behaviors/talos_teleop.hpp"

namespace inria_wbc
{
    namespace behaviors
    {
        static AutoRegister<TalosTeleop> __talos_squat("talos-teleop");

        TalosTeleop::TalosTeleop(const inria_wbc::controllers::TalosBaseController::Params &params) : Behavior(std::make_shared<inria_wbc::controllers::TalosPosTracking>(params))
        {
            //////////////////// INIT STACK OF TASK //////////////////////////////////////
            controller_ = std::make_shared<inria_wbc::controllers::TalosPosTracking>(params);
            //////////////////// INIT XSensTraj //////////////////////////////////////
            std::string teleoperation_file = "/home/user/talos_xsens/test1-004#S3.mvnx";
            int start_frame = 0;
            int end_frame = 6000;
            YAML::Node config = YAML::LoadFile(controller_->params().sot_config_path);
            inria_wbc::utils::parse(teleoperation_file, "teleoperation_file", config, false, "EXAMPLE");
            inria_wbc::utils::parse(start_frame, "start_frame", config, false, "EXAMPLE");
            inria_wbc::utils::parse(end_frame, "end_frame", config, false, "EXAMPLE");
            xsens_trajectory_ = std::make_shared<XSensJointTrajectory>(teleoperation_file, start_frame, end_frame);
            xsens_trajectory_->initialize(controller_->dt(), "Pelvis");

            //////////////////// Go To initial xsens position  //////////////////////////////////////
            lh_talos_init_ = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_se3_ref("lh");
            lh_talos_cmd_ = lh_talos_init_;

            rh_talos_init_ = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_se3_ref("rh");
            rh_talos_init_ = rh_talos_init_;

            floating_base_ = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->get_se3_ref("floatingb");

            lh_dhm_init_ = xsens_trajectory_->getDhmCurrentFramePin("left_hand");
            rh_dhm_init_ = xsens_trajectory_->getDhmCurrentFramePin("right_hand");
        }

        bool TalosTeleop::cmd(Eigen::VectorXd &q)
        {

            if (xsens_trajectory_->updateStep())
            {
                auto ref = xsens_trajectory_->getDhmCurrentFramePin("left_hand");
                lh_talos_cmd_.translation() = lh_talos_init_.translation() + ref.translation() - lh_dhm_init_.translation();
                lh_talos_cmd_.rotation() = ref.rotation();
                std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_se3_ref(lh_talos_cmd_, "lh");
                ref = xsens_trajectory_->getDhmCurrentFramePin("right_hand");
                rh_talos_cmd_.translation() = rh_talos_init_.translation() + ref.translation() - rh_dhm_init_.translation();
                rh_talos_cmd_.rotation() = ref.rotation();
                std::static_pointer_cast<inria_wbc::controllers::TalosPosTracking>(controller_)->set_se3_ref(rh_talos_cmd_, "rh");
                // std::cout << "lh_talos_cmd traj " << lh_talos_cmd.translation().transpose() << std::endl;
            }
            else
            {
                return false;
            }
            if (controller_->solve())
            {
                q.resize(controller_->q(false).size()); //size 50
                q = controller_->q(false);
                return true;
            }
            else
            {
                return false;
            }
        }

    } // namespace behaviors
} // namespace inria_wbc
