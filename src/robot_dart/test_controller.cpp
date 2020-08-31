
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <signal.h>
#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/robot.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

#include "inria_wbc/behaviors/factory.hpp"

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
    // take the name of the behavior as a argument
    std::string sot_config_path = argc > 1 ? argv[1] : "../etc/squat.yaml";
    std::cout << "using configuration:" << sot_config_path<<std::endl;
    // dt of the simulation and the controller
    float dt = 0.001;

    //////////////////// INIT DART ROBOT //////////////////////////////////////
    std::srand(std::time(NULL));
    std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
    auto robot = std::make_shared<robot_dart::Robot>("talos/talos.urdf", packages);
    robot->set_position_enforced(true);
    robot->set_actuator_types("torque");

    //////////////////// INIT DART SIMULATION WORLD //////////////////////////////////////
    robot_dart::RobotDARTSimu simu(dt);
    simu.set_collision_detector("fcl");

#ifdef GRAPHIC
    robot_dart::gui::magnum::GraphicsConfiguration configuration;
    configuration.width = 1280;
    configuration.height = 960;
    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(&simu, configuration);
    simu.set_graphics(graphics);
    graphics->look_at({3.5, -2, 2.2}, {0., 0., 1.4});
    graphics->record_video("talos.mp4");
#endif
    simu.add_robot(robot);
    simu.add_checkerboard_floor();

    //////////////////// INIT STACK OF TASK //////////////////////////////////////

    inria_wbc::controllers::TalosBaseController::Params params = {robot->model_filename(),
                                                                 "../etc/talos_configurations.srdf",
                                                                 sot_config_path,
                                                                 "",
                                                                 dt,
                                                                 false,
                                                                 robot->mimic_dof_names()};

    std::string behavior_name;
    YAML::Node config = YAML::LoadFile(sot_config_path);
    inria_wbc::utils::parse(behavior_name, "name", config, false, "BEHAVIOR");
    // params = inria_wbc::controllers::parse_params(config);
   
    auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, params);

    auto controller = behavior->controller();
    auto all_dofs = controller->all_dofs();
    auto controllable_dofs = controller->controllable_dofs();
    robot->set_positions(controller->q0(), all_dofs);
    uint ncontrollable = controllable_dofs.size();


    //////////////////// START SIMULATION //////////////////////////////////////
    simu.set_control_freq(1000); // 1000 Hz
    simu.set_graphics_freq(1000);
    
    // for benchmarking
    double time_simu = 0, time_cmd = 0;
    int it_simu = 0, it_cmd = 0;
    
    // the main loop
    using namespace std::chrono;
    Eigen::VectorXd q;
    while (simu.scheduler().next_time() < 20. && !simu.graphics()->done()) {
        // step the command
        if (simu.schedule(simu.control_freq())) {
            auto t1 = high_resolution_clock::now();
            bool solution_found = behavior->cmd(q);
            if(solution_found)
            {
                auto cmd = compute_spd(robot->skeleton(), q);
                robot->set_commands(controller->filter_cmd(cmd).tail(ncontrollable), controllable_dofs);
            }
            else
            {
                return -1;
            }
            auto t2 = high_resolution_clock::now();
            time_cmd += duration_cast<milliseconds>(t2 - t1).count();
            ++it_cmd;
        }
        // step the simulation
        {
            auto t1 = high_resolution_clock::now();
            simu.step_world();
            auto t2 = high_resolution_clock::now();
            time_simu += duration_cast<milliseconds>(t2 - t1).count();
            ++it_simu;
        }
        // print timing information
        if (it_simu == 100) {
            std::cout<<"Average time (iteration simu): "<<time_simu / it_simu << " ms"
                     << "\tAverage time(iteration command):"<< time_cmd / it_cmd << " ms"
                     << std::endl;;
            it_simu = 0;
            it_cmd = 0;
            time_cmd = 0;
            time_simu = 0;
            
        }
    }
    return 0;
}
