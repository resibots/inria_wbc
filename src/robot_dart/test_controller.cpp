
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <signal.h>
#include <boost/program_options.hpp>

#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/robot.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

#include "inria_wbc/behaviors/factory.hpp"

Eigen::VectorXd compute_spd(dart::dynamics::SkeletonPtr robot, const Eigen::VectorXd &targetpos)
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

Eigen::VectorXd compute_velocities(dart::dynamics::SkeletonPtr robot, const Eigen::VectorXd &targetpos, double dt)
{
    Eigen::VectorXd q = robot->getPositions();
    Eigen::VectorXd vel = (targetpos - q) / dt;
    return vel;
}

volatile sig_atomic_t stop;
void stopsig(int signum)
{
    stop = 1;
}
int main(int argc, char *argv[])
{
    // program options
    namespace po = boost::program_options;
    po::options_description desc("Test_controller options");
    // clang-format off
    desc.add_options()
    ("help,h", "produce help message")
    ("conf,c", po::value<std::string>()->default_value("../etc/squat.yaml"), "Configuration file of the tasks (yaml) [default: ../etc/squat.yaml]")
    ("fast,f", "fast (simplified) Talos [default: false]")
    ("big_window,b", "use a big window (nicer but slower) [default:true]")
    ("actuators,a", po::value<std::string>()->default_value("torque"), "actuator model torque/velocity/servo (always for position control) [default:torque]")
    ("enforce_position,e", po::value<bool>()->default_value(true), "enforce the positions of the URDF [default:true]")
    ("collision,k", po::value<std::string>()->default_value("fcl"), "collision engine [default:fcl]")
    ("video,v", po::value<std::string>(), "save the display to a video [filename]")
    ("duration,d", po::value<int>()->default_value(20), "duration in seconds [20]")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 0;
    }

    // clang-format off
    std::cout<< "------ CONFIGURATION ------" << std::endl;
    for (const auto& kv : vm){
        std::cout << kv.first << " ";
        try { std::cout << kv.second.as<std::string>();
        } catch(...) {/* do nothing */ }
        try { std::cout << kv.second.as<bool>();
        } catch(...) {/* do nothing */ }
        try { std::cout << kv.second.as<int>();
        } catch(...) {/* do nothing */ }
        std::cout<< std::endl;
    }
    std::cout << "--------------------------" << std::endl;
    // clang-format on


    // dt of the simulation and the controller
    float dt = 0.001;
    std::cout << "dt:" << dt << std::endl;

    //////////////////// INIT DART ROBOT //////////////////////////////////////
    std::srand(std::time(NULL));
    std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
    std::string urdf = vm.count("fast") ? "talos/talos_fast.urdf" : "talos/talos.urdf";
    auto robot = std::make_shared<robot_dart::Robot>(urdf, packages);
    robot->set_position_enforced(vm["enforce_position"].as<bool>());
    robot->set_actuator_types(vm["actuators"].as<std::string>());

    //////////////////// INIT DART SIMULATION WORLD //////////////////////////////////////
    robot_dart::RobotDARTSimu simu(dt);
    simu.set_collision_detector(vm["collision"].as<std::string>());

#ifdef GRAPHIC
    robot_dart::gui::magnum::GraphicsConfiguration configuration;
    if (vm.count("big_window"))
    {
        configuration.width = 1280;
        configuration.height = 960;
    }
    else
    {
        configuration.width = 800;
        configuration.height = 500;        
    }
    
    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(&simu, configuration);
    simu.set_graphics(graphics);
    graphics->look_at({3.5, -2, 2.2}, {0., 0., 1.4});
    if (vm.count("video"))
        graphics->record_video(vm["video"].as<std::string>());
#endif
    simu.add_robot(robot);
    simu.add_checkerboard_floor();

    //////////////////// INIT STACK OF TASK //////////////////////////////////////
    std::string sot_config_path = vm["conf"].as<std::string>();
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

    auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, params);

    auto controller = behavior->controller();
    auto all_dofs = controller->all_dofs();
    auto controllable_dofs = controller->controllable_dofs();

    robot->set_positions(controller->q0(), all_dofs);

    uint ncontrollable = controllable_dofs.size();

    //////////////////// START SIMULATION //////////////////////////////////////
    simu.set_control_freq(1000); // 1000 Hz
    double time_simu = 0, time_cmd = 0;
    int it_simu = 0, it_cmd = 0;

    // the main loop
    using namespace std::chrono;
    while (simu.scheduler().next_time() < vm["duration"].as<int>() && !simu.graphics()->done())
    {
        // step the command
        if (simu.schedule(simu.control_freq()))
        {
            auto t1 = high_resolution_clock::now();
            bool solution_found = behavior->update();
            auto q = controller->q(false);
            if (solution_found)
            {
                Eigen::VectorXd cmd;
                if (vm["actuators"].as<std::string>() == "velocity" || vm["actuators"].as<std::string>() == "servo")
                    cmd = compute_velocities(robot->skeleton(), q, dt);
                else // torque
                    cmd = compute_spd(robot->skeleton(), q);
                robot->set_commands(controller->filter_cmd(cmd).tail(ncontrollable), controllable_dofs);
            }
            else
            {
                std::cerr << "Solver failed! aborting" << std::endl;
                return -1;
            }
            auto t2 = high_resolution_clock::now();
            time_cmd += duration_cast<microseconds>(t2 - t1).count();
            ++it_cmd;
        }
        // step the simulation
        {
            auto t1 = high_resolution_clock::now();
            simu.step_world();  
            auto t2 = high_resolution_clock::now();
            time_simu += duration_cast<microseconds>(t2 - t1).count();
            ++it_simu;
        }
        // print timing information
        if (it_simu == 1000)
        {
            std::cout << "Average time (iteration simu): " << time_simu / it_simu / 1000. << " ms"
                      << "\tAverage time(iteration command):" << time_cmd / it_cmd / 1000. << " ms"
                      << std::endl;
            ;
            it_simu = 0;
            it_cmd = 0;
            time_cmd = 0;
            time_simu = 0;
        }
    }
    return 0;
}
