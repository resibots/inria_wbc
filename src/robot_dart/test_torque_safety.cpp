
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <signal.h>
#include <boost/program_options.hpp>

#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/robot.hpp>
#include <robot_dart/sensor/torque.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

#include "inria_wbc/behaviors/factory.hpp"
#include "inria_wbc/utils/torque_safety.hpp"

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


Eigen::VectorXd vector_select(const Eigen::VectorXd& vec, const std::vector<int>& ids)
{
    int ct = 0;
    Eigen::VectorXd new_vec(ids.size());
    for(int value : ids)
        new_vec(ct++) = vec(value);
    return new_vec;
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

    robot->set_cfriction_coeffs(0.0);
    robot->set_damping_coeffs(0.0);
    
    //robot->set_actuator_types("velocity");

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


    ///////////////// TORQUE COLLISION SAFETY CHECK ////////////////////////////
    for(auto a : robot->body_names())
        std::cout << a << std::endl;
    robot->set_draw_axis("arm_left_2_link");
    robot->set_draw_axis("arm_left_4_link");
    
    // add joint torque sensor to the simulation
    std::vector<std::string> joints_with_tq = 
        {"leg_left_1_joint", "leg_left_2_joint", "leg_left_3_joint", "leg_left_4_joint", "leg_left_5_joint", "leg_left_6_joint", 
        "leg_right_1_joint", "leg_right_2_joint", "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint", "leg_right_6_joint",
        "torso_1_joint", "torso_2_joint", 
        "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint",
        "arm_right_1_joint", "arm_right_2_joint","arm_right_3_joint", "arm_right_4_joint"}; // id 17 left_arm_4

    Eigen::VectorXd torque_threshold(joints_with_tq.size());
    torque_threshold << 3.5e+00, 3.9e+01, 2.9e+01, 4.4e+01, 5.7e+01, 2.4e+01, 
                3.5e+00, 3.9e+01, 2.9e+01, 4.4e+01, 5.7e+01, 2.4e+01, 
                4.8e-04, 1.6e-01, 
                4.5e-02, 2.6e-02, 6.1e-03, 9.0e-03, 
                4.5e-02, 2.6e-02, 6.1e-03, 9.0e-03;

    torque_threshold << 3.5e+05, 3.9e+05, 2.9e+05, 4.4e+05, 5.7e+05, 2.4e+05, 
            3.5e+05, 3.9e+05, 2.9e+05, 4.4e+05, 5.7e+05, 2.4e+05, 
            5e-02, 1e-01, 
            5e-02, 5e-02,5e-02, 8e-02, 
            5e-02, 5e-02, 5e-02, 8e-02;

    TorqueCollisionDetection torque_collision(joints_with_tq, torque_threshold, 5);
    torque_collision.set_ignore_count(5);
    
    std::vector<int> tq_joints_idx;
    std::vector<std::shared_ptr<robot_dart::sensor::Torque>> torque_sensors;

    for(const auto& joint : joints_with_tq)
    {
        tq_joints_idx.push_back(robot->dof_index(joint));
        torque_sensors.push_back(simu.add_sensor<robot_dart::sensor::Torque>(&simu, robot, joint, 1000));
    }

    // reading from sensors
    Eigen::VectorXd tq_sensors = Eigen::VectorXd::Zero(joints_with_tq.size());
    // expected torque from tsid
    Eigen::VectorXd tsid_tau = Eigen::VectorXd::Zero(joints_with_tq.size());


    Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", "\n", "", "");
    std::cout.precision(7);

    //////////////////// START SIMULATION //////////////////////////////////////
    simu.set_control_freq(1000); // 1000 Hz
    double time_simu = 0, time_cmd = 0;
    int it_simu = 0, it_cmd = 0;
    int sec = 0;

    bool external_applied = false;
    bool external_detected = false;

    // the main loop
    using namespace std::chrono;
    while (simu.scheduler().next_time() < vm["duration"].as<int>() && !simu.graphics()->done())
    {
        // step the command
        if (simu.schedule(simu.control_freq()))
        {
            auto t1 = high_resolution_clock::now();

            // get actual torque from sensors
            for(auto tq_sens = torque_sensors.cbegin(); tq_sens < torque_sensors.cend(); ++tq_sens)
                tq_sensors(std::distance(torque_sensors.cbegin(), tq_sens)) = (*tq_sens)->torques()(0, 0);


            // safety check on torque sensors (check prevous command with actual sensor measurement)
            if(vm["actuators"].as<std::string>() == "velocity" && it_cmd > 0)
            {
                
                if( !torque_collision.check(tsid_tau, tq_sensors, TorqueCollisionDetection::MedianFilter() ) )
                {
                    std::cerr << "torque discrepancy over threshold: " <<torque_collision.get_discrepancy().maxCoeff() << '\n';
                    auto inv = torque_collision.get_invalid_ids();
                    
                    std::cerr <<"invalid dofs: [";
                    for(auto v : inv) std::cerr << v <<" ";
                    std::cerr <<"] with discrepancy [";
                    for(auto v : inv) std::cerr << torque_collision.get_discrepancy()(v) <<" ";
                    std::cerr << "]\n";

                    external_detected = true;
                }
                else
                {
                    external_detected = false;
                }

                Eigen::VectorXd output_data(tsid_tau.size() + tq_sensors.size());
                output_data << tsid_tau, torque_collision.get_filtered_sensors(); //tq_sensors;
                std::cout << output_data.transpose().format(fmt) << std::endl;
            }

            {
                std::cerr << "Collision detection [";
                if(external_applied && external_detected) std::cerr << "\033[32mOK";          // green (true positive)
                else if(!external_applied && external_detected) std::cerr << "\033[33mKO";    // yellow (false positive)
                else if(external_applied && !external_detected) std::cerr << "\033[31mKO";    // red (false negative)
                else if(!external_applied && !external_detected) std::cerr << "\033[37mOK";   // white (true negative)
                std::cerr << "\033[37m]" << std::endl;
            }

            bool solution_found = behavior->update();
            auto q = controller->q(false);
            if (solution_found)
            {
                Eigen::VectorXd cmd;
                if (vm["actuators"].as<std::string>() == "velocity" || vm["actuators"].as<std::string>() == "servo")
                {
                    cmd = compute_velocities(robot->skeleton(), q, dt);
                }
                else // torque
                {
                    cmd = compute_spd(robot->skeleton(), q);
                }
                
                //cmd(tq_joints_idx[15]) = 0;
                //cmd(tq_joints_idx[16]) = 0;
                //cmd(tq_joints_idx[17]) = 0;
                //std::cerr << "\n\ncommand: " << cmd.transpose() << std::endl;

                auto cmd_filtered = controller->filter_cmd(cmd).tail(ncontrollable);
                //std::cerr << "cmd: " << cmd_filtered.transpose() << std::endl;
                robot->set_commands(cmd_filtered, controllable_dofs);


                // std::cerr << "commands: " << std::endl << vector_select(cmd, tq_joints_idx).transpose() << std::endl;

                // update the expected torque from tsid solution
                tsid_tau = vector_select(controller->tau(), tq_joints_idx);

            }
            else
            {
                std::cerr << "Solver failed! aborting" << std::endl;
                return -1;
            }
            auto t2 = high_resolution_clock::now();
            time_cmd += duration_cast<microseconds>(t2 - t1).count();
            ++it_cmd;

            std::set<int> check_sec = {2, 3, 9, 15, 16};

            // apply some external forces
            if(check_sec.count(sec) != 0)
            {
                //std::cerr << "external force applied " <<it_cmd << '\n';
                robot->set_external_force("arm_left_2_link", Eigen::Vector3d(0.0, 5.0, 0.0), Eigen::Vector3d(0.0, 0.0, -0.30), true);
                external_applied = true;
            }
            else
            {
                // std::cerr << "zero external force " << it_cmd << '\n';
                robot->set_external_force("arm_left_2_link", Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, -0.30), true);
                external_applied = false;
            }
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
            //std::cout << "Average time (iteration simu): " << time_simu / it_simu / 1000. << " ms"
            //          << "\tAverage time(iteration command):" << time_cmd / it_cmd / 1000. << " ms"
            //          << std::endl;

            it_simu = 0;
            it_cmd = 0;
            time_cmd = 0;
            time_simu = 0;
            ++sec;
        }
    }
    return 0;
}
