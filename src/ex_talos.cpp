
#include <algorithm>
#include <cstdlib>
#include <iostream>

#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/robot.hpp>

#include "tsid_sot_talos.hpp"


#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

int main()
{

    //////////////////// INIT STACK OF TASK //////////////////////////////////////
    float dt = 0.001;
    int duration = 20 / dt;
    float arm_speed = 0.05;
    tsid_sot::Talos::Params params = {"../res/models/talos.urdf",
                                          "../res/models/talos_configurations.srdf",
                                          dt};
    auto talos_sot = tsid_sot::Talos(params, "../res/yaml/sot.yaml", true);
    auto all_dofs = talos_sot.all_dofs();
    auto controllable_dofs = talos_sot.controllable_dofs();
    uint ncontrollable = controllable_dofs.size();
    Eigen::VectorXd cmd = Eigen::VectorXd::Zero(ncontrollable);

    //////////////////// INIT DART ROBOT //////////////////////////////////////
    std::srand(std::time(NULL));
    std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
    auto global_robot = std::make_shared<robot_dart::Robot>("talos/talos.urdf", packages);
    global_robot->skeleton()->setPosition(5, 1.1);
    global_robot->skeleton()->setPosition(2, 1.57);
    global_robot->set_position_enforced(true);
    // Set actuator types to VELOCITY motors so that they stay in position without any controller
    global_robot->set_actuator_types(dart::dynamics::Joint::VELOCITY);
    // First 6-DOFs should always be FORCE if robot is floating base
    for (size_t i = 0; i < 6; i++)
        global_robot->set_actuator_type(i, dart::dynamics::Joint::FORCE);
    global_robot->set_positions(talos_sot.q0(), all_dofs);

    //////////////////// INIT DART SIMULATION WORLD //////////////////////////////////////
    robot_dart::RobotDARTSimu simu(dt);
    simu.set_collision_detector("fcl");

#ifdef GRAPHIC
    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(&simu);
    simu.set_graphics(graphics);
    graphics->look_at({0., 3.5, 2.}, {0., 0., 0.25});
    graphics->record_video("talos.mp4");
#endif
    simu.add_robot(global_robot);
    simu.add_checkerboard_floor();

    //////////////////// PLAY SIMULATION //////////////////////////////////////
    for (int i = 0; i < duration; i++)
    {
        talos_sot.add_to_lh_ref(0.0, 0.0, arm_speed * dt * sin(duration / 100));
        talos_sot.solve();
        cmd = talos_sot.dq().tail(ncontrollable);
        global_robot->set_commands(cmd, controllable_dofs);
        simu.step_world();
    }

    global_robot.reset();
    return 0;
}
