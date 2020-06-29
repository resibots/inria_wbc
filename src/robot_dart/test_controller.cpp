
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <signal.h>
#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/robot.hpp>

#include "examples/factory.hpp"
#include "examples/talos_squat.hpp"

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

Eigen::VectorXd compute_spd(dart::dynamics::SkeletonPtr robot, Eigen::VectorXd targetpos)
{
    Eigen::VectorXd q = robot->getPositions();
    Eigen::VectorXd dq = robot->getVelocities();

    float stiffness = 10000;
    float damping = 100;
    int ndofs = robot->getNumDofs();
    Eigen::MatrixXd Kp = Eigen::MatrixXd::Identity(ndofs, ndofs);
    Eigen::MatrixXd Kd = Eigen::MatrixXd::Identity(ndofs, ndofs);

    for (std::size_t i = 0; i < robot->getNumDofs(); ++i)
    {
        Kp(i, i) = stiffness;
        Kd(i, i) = damping;
    }
    for (std::size_t i = 0; i < 6; ++i)
    {
        Kp(i, i) = 0;
        Kd(i, i) = 0;
    }

    Eigen::MatrixXd invM = (robot->getMassMatrix() + Kd * robot->getTimeStep()).inverse();
    Eigen::VectorXd p = -Kp * (q + dq * robot->getTimeStep() - targetpos);
    Eigen::VectorXd d = -Kd * dq;
    Eigen::VectorXd qddot = invM * (-robot->getCoriolisAndGravityForces() + p + d + robot->getConstraintForces());
    Eigen::VectorXd commands = p + d - Kd * qddot * robot->getTimeStep();
    return commands;
}
volatile sig_atomic_t stop;
void stopsig(int signum)
{
    stop = 1;
}
int main(int argc, char *argv[])
{
    //////////////////// INIT DART ROBOT //////////////////////////////////////
    std::srand(std::time(NULL));
    std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
    auto robot = std::make_shared<robot_dart::Robot>("talos/talos.urdf", packages);
    robot->set_position_enforced(true);
    // Set actuator types to VELOCITY motors so that they stay in position without any controller
    robot->set_actuator_types(dart::dynamics::Joint::FORCE);
    // First 6-DOFs should always be FORCE if robot is floating base
    // for (size_t i = 0; i < 6; i++)
    //     robot->set_actuator_type(i, dart::dynamics::Joint::FORCE);

    //////////////////// INIT STACK OF TASK //////////////////////////////////////
    float dt = 0.001;
    int duration = 20 / dt;
    float arm_speed = 0.05;
    tsid_sot::controllers::TalosBaseController::Params params = {robot->model_filename(),
                                                                 "../etc/models/talos_configurations.srdf",
                                                                 dt};
    std::string sot_config_path = "../etc/yaml/sot.yaml";
    std::string example_name;
    YAML::Node config = YAML::LoadFile(sot_config_path);
    tsid_sot::utils::parse(example_name, "example_name_", config, false, "EXAMPLE");
    auto example = tsid_sot::example::ExampleFactory::instance().create_example(example_name, params, sot_config_path, "", robot->mimic_dof_names(), false);
    // auto example = std::make_shared<tsid_sot::example::TalosSquat>(params, "../etc/yaml/sot-squat.yaml", "", robot->mimic_dof_names());

    auto controller = example->controller();
    auto all_dofs = controller->all_dofs();
    auto controllable_dofs = controller->controllable_dofs();
    robot->set_positions(controller->q0(), all_dofs);
    uint ncontrollable = controllable_dofs.size();

    //////////////////// INIT DART SIMULATION WORLD //////////////////////////////////////
    robot_dart::RobotDARTSimu simu(dt);
    simu.set_collision_detector("fcl");

#ifdef GRAPHIC
    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(&simu);
    simu.set_graphics(graphics);
    graphics->look_at({0., 3.5, 2.}, {0., 0., 0.25});
    graphics->record_video("talos.mp4");
#endif
    simu.add_robot(robot);
    simu.add_checkerboard_floor();

    //////////////////// PLAY SIMULATION //////////////////////////////////////
    signal(SIGINT, stopsig); // to stop the simulation loop
    while (!stop)
    {
        auto cmd = compute_spd(robot->skeleton(), example->cmd());
        robot->set_commands(controller->filter_cmd(cmd).tail(ncontrollable), controllable_dofs);
        simu.step_world();
    }

    robot.reset();
    return 0;
}
