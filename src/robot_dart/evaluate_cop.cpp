
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include <signal.h>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

#include "inria_wbc/behaviors/behavior.hpp"

void evaluate_cop(const Eigen::Vector6d& lf_torque_force, const Eigen::Vector6d& rf_torque_force)
{
    Eigen::Vector2d CoP(0., 0.);
    double ankle_height_ = 0.114;

    Eigen::Vector3d Rft;
    Eigen::Vector3d Lft;
    Rft << 0.0, -0.118, 0.0;
    Lft << 0.0, 0.116, 0.0;

    // CoP left foot
    CoP(0) = (-lf_torque_force(1) - ankle_height_ * lf_torque_force(3)) / lf_torque_force(5);
    CoP(1) = (lf_torque_force(0) - ankle_height_ * lf_torque_force(4)) / lf_torque_force(5);
    CoP(0) = CoP(0) + Lft(0); //Lft_pos(0);
    CoP(1) = CoP(1) + Lft(1); //Lft_pos(1);
    Eigen::Vector3d lcop_raw;
    lcop_raw << CoP(0), CoP(1), 0.0;

    //CoP right foot
    CoP(0) = (-rf_torque_force(1) - ankle_height_ * rf_torque_force(3)) / rf_torque_force(5);
    CoP(1) = (rf_torque_force(0) - ankle_height_ * rf_torque_force(4)) / rf_torque_force(5);
    CoP(0) = CoP(0) + Rft(0); //Lft_pos(0);
    CoP(1) = CoP(1) + Rft(1); // Lft_pos(1);
    Eigen::Vector3d rcop_raw;
    rcop_raw << CoP(0), CoP(1), 0.0;

    double Fz_ratio_l = lf_torque_force(5) / (lf_torque_force(5) + rf_torque_force(5));
    double Fz_ratio_r = rf_torque_force(5) / (lf_torque_force(5) + rf_torque_force(5));

    Eigen::Vector3d cop_in_lft_raw;
    Eigen::Vector3d cop_in_rft_raw;
    cop_in_lft_raw = Fz_ratio_l * lcop_raw + Fz_ratio_r * (rcop_raw + Rft - Lft);
    cop_in_rft_raw = Fz_ratio_l * (lcop_raw + Lft - Rft) + Fz_ratio_r * rcop_raw;

    Eigen::Vector3d cop_delta;
    cop_delta = cop_in_lft_raw * Fz_ratio_l + cop_in_rft_raw * Fz_ratio_r;
    std::cout << "Global raw CoP :   x : " << cop_delta(0) << "  y : " << cop_delta(1) << std::endl;
}

volatile sig_atomic_t stop;
void stopsig(int signum)
{
    stop = 1;
}
int main(int argc, char* argv[])
{
    // take the name of the behavior as a argument
    std::string sot_config_path = argc > 1 ? argv[1] : "../etc/squat.yaml";
    std::cout << "using configuration:" << sot_config_path << std::endl;
    // dt of the simulation and the controller
    float dt = 0.001;

    //////////////////// INIT DART ROBOT //////////////////////////////////////
    std::srand(std::time(NULL));
    std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
    auto robot = std::make_shared<robot_dart::Robot>("talos/talos.urdf", packages);
    robot->set_position_enforced(true);
    robot->set_actuator_types("velocity");

    //////////////////// INIT DART SIMULATION WORLD //////////////////////////////////////
    robot_dart::RobotDARTSimu simu(dt);
    simu.set_collision_detector("fcl");

#ifdef GRAPHIC
    robot_dart::gui::magnum::GraphicsConfiguration configuration;
    configuration.width = 1280;
    configuration.height = 960;
    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(configuration);
    simu.set_graphics(graphics);
    graphics->look_at({3.5, -2, 2.2}, {0., 0., 1.4});
#endif
    simu.add_robot(robot);
    simu.add_checkerboard_floor();

    //////////////////// INIT STACK OF TASK //////////////////////////////////////

    inria_wbc::controllers::Controller::Params params = {robot->model_filename(),
        "../etc/talos_configurations.srdf",
        sot_config_path,
        "",
        dt,
        false,
        robot->mimic_dof_names()};

    std::string behavior_name, controller_name;
    YAML::Node config = YAML::LoadFile(sot_config_path);
    inria_wbc::utils::parse(behavior_name, "name", config, "BEHAVIOR", false);
    inria_wbc::utils::parse(controller_name, "name", config, "CONTROLLER", false);

    auto controller = inria_wbc::controllers::Factory::instance().create(controller_name, params);
    auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, controller);

    auto all_dofs = controller->all_dofs();
    auto controllable_dofs = controller->controllable_dofs();
    robot->set_positions(controller->q0(), all_dofs);
    uint ncontrollable = controllable_dofs.size();
    std::pair<Eigen::Vector6d, Eigen::Vector6d> lf_torque_force;
    std::pair<Eigen::Vector6d, Eigen::Vector6d> rf_torque_force;

    //////////////////// START SIMULATION //////////////////////////////////////
    simu.set_control_freq(1000); // 1000 Hz
    while (!simu.graphics()->done()) {
        if (simu.schedule(simu.control_freq())) {
            lf_torque_force = robot->force_torque(robot->joint_index("leg_left_6_joint"));
            rf_torque_force = robot->force_torque(robot->joint_index("leg_right_6_joint"));
            evaluate_cop(lf_torque_force.first, rf_torque_force.first);
        }
        simu.step_world();
    }

    return 0;
}
