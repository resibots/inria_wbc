
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

Eigen::VectorXd compute_spd(dart::dynamics::SkeletonPtr robot, Eigen::VectorXd targetpos)
{
    Eigen::VectorXd q = robot->getPositions();
    Eigen::VectorXd dq = robot->getVelocities();

    float stiffness = 10000;
    float damping = 100;
    int ndofs = robot->getNumDofs();
    Eigen::MatrixXd Kp = Eigen::MatrixXd::Identity(ndofs, ndofs);
    Eigen::MatrixXd Kd = Eigen::MatrixXd::Identity(ndofs, ndofs);

    for (std::size_t i = 0; i < robot->getNumDofs(); ++i) {
        Kp(i, i) = stiffness;
        Kd(i, i) = damping;
    }
    for (std::size_t i = 0; i < 6; ++i) {
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
    robot->skeleton()->setPosition(5, 1.1);
    robot->skeleton()->setPosition(2, 1.57);
    robot->set_actuator_types("velocity");

    //////////////////// INIT DART SIMULATION WORLD //////////////////////////////////////
    robot_dart::RobotDARTSimu simu(dt);
    simu.set_collision_detector("bullet");
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
	true,
        "",
        dt,
        false,
        robot->mimic_dof_names()};

    std::string controller_name;
    YAML::Node config = YAML::LoadFile(sot_config_path);
    inria_wbc::utils::parse(controller_name, "name", config, "CONTROLLER", true);

    auto controller = inria_wbc::controllers::Factory::instance().create(controller_name, params);

    auto masses = controller->pinocchio_model_masses();
    std::vector<std::string> joint_names = controller->pinocchio_joint_names();

    std::cout << "PINOCCHIO" << std::endl;
    double total_mass_pin = 0.0;
    for (uint i = 0; i < masses.size(); i++) {
        std::cout << "mass : " << joint_names[i] << " = " << masses[i] << std::endl;
        total_mass_pin += masses[i];
    }
    std::cout << "total mass : " << total_mass_pin << std::endl;

    std::cout << "\nDART" << std::endl;
    double total_mass = 0.0;
    for (auto& s : robot->body_names()) {
        std::cout << "mass: " << s << " = " << robot->body_mass(s) << std::endl;
        total_mass += robot->body_mass(s);
    }
    std::cout << "total mass : " << total_mass << std::endl;

    return 0;
}
