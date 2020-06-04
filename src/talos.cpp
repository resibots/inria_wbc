#include "tsid-sot.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>

#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot_dart_simu.hpp>

#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

int main()
{

    float dt = 0.001;
    int duration = 20 / dt;
    float arm_speed = 0.01;
    tsid_sot::talos_sot::Params params = {"../res/models/talos.urdf",
                                          "../res/models/talos_configurations.srdf",
                                          dt};
    auto controller = tsid_sot::talos_sot(params);

    std::srand(std::time(NULL));
    std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "/home/user/rf_ws/src/talos_robot/talos_description"}};
    auto global_robot = std::make_shared<robot_dart::Robot>(params.urdf_path, packages);

    global_robot->set_position_enforced(true);

    // Set actuator types to VELOCITY motors so that they stay in position without any controller
    global_robot->set_actuator_types(dart::dynamics::Joint::VELOCITY);
    // First 6-DOFs should always be FORCE if robot is floating base
    for (size_t i = 0; i < 6; i++)
        global_robot->set_actuator_type(i, dart::dynamics::Joint::FORCE);

    robot_dart::RobotDARTSimu simu(dt);
    simu.world()->getConstraintSolver()->setCollisionDetector(dart::collision::FCLCollisionDetector::create());
#ifdef GRAPHIC
    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics<>>(&simu);
    simu.set_graphics(graphics);
    graphics->look_at({0., 3.5, 2.}, {0., 0., 0.25});
#endif

    auto all_dofs = controller.all_dofs();
    auto controllable_dofs = controller.controllable_dofs();
    uint ncontrollable = controllable_dofs.size();
    Eigen::VectorXd cmd = Eigen::VectorXd::Zero(ncontrollable);

    global_robot->set_positions(controller.q0(), all_dofs);
    simu.add_robot(global_robot);
    simu.add_checkerboard_floor();

    for (int i = 0; i < duration; i++)
    {
        controller.add_to_lh_ref(0.0, 0.0, arm_speed * dt);
        controller.solve();

        cmd = controller.dq().tail(ncontrollable);
        global_robot->set_commands(cmd, controllable_dofs);
        simu.step_world();
    }

    global_robot.reset();
    return 0;
}
