
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <signal.h>
#include <boost/program_options.hpp>

#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/sensor/force_torque.hpp>
#include <robot_dart/sensor/torque.hpp>
#include <robot_dart/sensor/imu.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

#include "inria_wbc/behaviors/behavior.hpp"
#include "inria_wbc/exceptions.hpp"
#include "inria_wbc/robot_dart/cmd.hpp"
#include "inria_wbc/safety/torque_safety.hpp"


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
    ("actuators,a", po::value<std::string>()->default_value("servo"), "actuator model torque/velocity/servo (always for position control) [default:torque]")
    ("enforce_position,e", po::value<bool>()->default_value(true), "enforce the positions of the URDF [default:true]")
    ("collision,k", po::value<std::string>()->default_value("fcl"), "collision engine [default:fcl]")
    ("mp4,m", po::value<std::string>(), "save the display to a mp4 video [filename]")
    ("duration,d", po::value<int>()->default_value(20), "duration in seconds [20]")
    ("ghost,g", "display the ghost (Pinocchio model)")
    ("push,p", po::value<std::vector<float>>(), "push the robot at t=x1 0.25 s")
    ("verbose,v", "verbose mode (controller)")
    ("log,l", po::value<std::vector<std::string>>()->default_value(std::vector<std::string>(),""), 
        "log the trajectory of a dart body [with urdf names] or timing or CoM or cost, example: -l timing -l com -l lf -l cost_com -l cost_lf")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::clog << desc << std::endl;
        return 0;
    }

    bool verbose = (vm.count("verbose") != 0);

    // clang-format off
    std::clog<< "------ CONFIGURATION ------" << std::endl;
    for (const auto& kv : vm){
        std::clog << kv.first << " ";
        try { std::clog << kv.second.as<std::string>();
        } catch(...) {/* do nothing */ }
        try { std::clog << kv.second.as<bool>();
        } catch(...) {/* do nothing */ }
        try { std::clog << kv.second.as<int>();
        } catch(...) {/* do nothing */ }
        std::clog<< std::endl;
    }
    std::clog << "--------------------------" << std::endl;
    // clang-format on


    // dt of the simulation and the controller
    float dt = 0.001;
    std::clog << "dt:" << dt << std::endl;

    //////////////////// INIT DART ROBOT //////////////////////////////////////
    std::srand(std::time(NULL));
    std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
    std::string urdf = vm.count("fast") ? "talos/talos_fast.urdf" : "talos/talos.urdf";
    auto robot = std::make_shared<robot_dart::Robot>(urdf, packages);
    robot->set_position_enforced(vm["enforce_position"].as<bool>());
    robot->set_actuator_types(vm["actuators"].as<std::string>());

    //robot->set_friction_coeffs(0.0);
    //robot->set_damping_coeffs(0.0);
    
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

    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(configuration);
    simu.set_graphics(graphics);
    graphics->look_at({3.5, -2, 2.2}, {0., 0., 1.4});
    if (vm.count("video"))
        graphics->record_video(vm["video"].as<std::string>());
#endif
    simu.add_robot(robot);
    simu.add_checkerboard_floor();
    

    //////////////////// INIT STACK OF TASK //////////////////////////////////////
    std::string sot_config_path = vm["conf"].as<std::string>();
    inria_wbc::controllers::Controller::Params params = {
        robot->model_filename(),
        sot_config_path,
        dt,
        verbose,
        robot->mimic_dof_names()};

    std::string behavior_name, controller_name;
    YAML::Node config = YAML::LoadFile(sot_config_path);
    inria_wbc::utils::parse(behavior_name, "name", config, "BEHAVIOR", verbose);
    inria_wbc::utils::parse(controller_name, "name", config, "CONTROLLER", verbose);

    auto controller = inria_wbc::controllers::Factory::instance().create(controller_name, params);
    auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, controller);
    assert(behavior);

    auto all_dofs = controller->all_dofs();
    auto floating_base = all_dofs;
    floating_base.resize(6);

    auto controllable_dofs = controller->controllable_dofs();
    robot->set_positions(controller->q0(), all_dofs);
    
    uint ncontrollable = controllable_dofs.size();



    ///////////////// TORQUE COLLISION SAFETY CHECK ////////////////////////////
    for(auto a : robot->body_names())
        std::clog << a << std::endl;
    robot->set_draw_axis("arm_left_2_link");
    robot->set_draw_axis("arm_left_4_link");
    robot->set_draw_axis("arm_right_4_link");
    robot->set_draw_axis("arm_right_4_link");
    robot->set_draw_axis("torso_2_link");
    robot->set_draw_axis("leg_left_3_link");
    robot->set_draw_axis("leg_right_4_link");
    robot->set_draw_axis("head_2_link");
    
    // add joint torque sensor to the simulation
    std::vector<std::string> joints_with_tq = 
        {"leg_left_1_joint", "leg_left_2_joint", "leg_left_3_joint", "leg_left_4_joint", "leg_left_5_joint", "leg_left_6_joint", 
        "leg_right_1_joint", "leg_right_2_joint", "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint", "leg_right_6_joint",
        "torso_1_joint", "torso_2_joint", 
        "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint",
        "arm_right_1_joint", "arm_right_2_joint","arm_right_3_joint", "arm_right_4_joint"}; // id 17 left_arm_4

    Eigen::VectorXd torque_threshold(joints_with_tq.size());
/*     torque_threshold << 3.5e+00, 3.9e+01, 2.9e+01, 4.4e+01, 5.7e+01, 2.4e+01, 
                3.5e+00, 3.9e+01, 2.9e+01, 4.4e+01, 5.7e+01, 2.4e+01, 
                4.8e-04, 1.6e-01, 
                4.5e-02, 2.6e-02, 6.1e-03, 9.0e-03, 
                4.5e-02, 2.6e-02, 6.1e-03, 9.0e-03; */

    torque_threshold << 3.5e+05, 3.9e+05, 2.9e+05, 4.4e+05, 5.7e+05, 2.4e+05, 
            3.5e+05, 3.9e+05, 2.9e+05, 4.4e+05, 5.7e+05, 2.4e+05, 
            5e-02, 2e-01, 
            5e-02, 5e-02, 5e-02, 1e-01, 
            5e-02, 5e-02, 5e-02, 1e-01;

    inria_wbc::safety::TorqueCollisionDetection torque_collision(torque_threshold, 10);
    torque_collision.set_max_consecutive_invalid(5);

    // add sensors to the robot
    auto ft_sensor_left = simu.add_sensor<robot_dart::sensor::ForceTorque>(robot, "leg_left_6_joint");
    auto ft_sensor_right = simu.add_sensor<robot_dart::sensor::ForceTorque>(robot, "leg_right_6_joint");
    robot_dart::sensor::IMUConfig imu_config;
    imu_config.body = robot->body_node("imu_link"); // choose which body the sensor is attached to
    imu_config.frequency = 1000; // update rate of the sensor
    auto imu = simu.add_sensor<robot_dart::sensor::IMU>(imu_config);
    
    std::vector<int> tq_joints_idx;
    std::vector<std::shared_ptr<robot_dart::sensor::Torque>> torque_sensors;

    auto filtered_dof_names = robot->dof_names(true, true, true); // filter out mimic, passive and locked dofs
    for(const auto& joint : joints_with_tq)
    {
        auto it = std::find(filtered_dof_names.begin(), filtered_dof_names.end(), joint);
        tq_joints_idx.push_back(std::distance(filtered_dof_names.begin(), it));
        torque_sensors.push_back(simu.add_sensor<robot_dart::sensor::Torque>(robot, joint, 1000)); 
        std::cerr << "Add joint " << joint << " with index " << std::distance(filtered_dof_names.begin(), it) << std::endl;
    }

    std::cerr << "\nDOF NAMES\n";
    for(const auto& i : robot->dof_names())
        std::cerr << i << std::endl;

    std::cerr << "\nFILTERED DOF NAMES\n";
    for(const auto& i : robot->dof_names(true, true, true))
        std::cerr << i << std::endl;

    std::cerr << "\nJOINT NAMES\n";
    for(const auto& i : robot->joint_names())
        std::cerr << i << std::endl;
    std::cerr << std::endl;
    
    // reading from sensors
    Eigen::VectorXd tq_sensors = Eigen::VectorXd::Zero(joints_with_tq.size());
    // expected torque from tsid
    Eigen::VectorXd tsid_tau = Eigen::VectorXd::Zero(joints_with_tq.size());


    // set of external forxes applied during simulation
    typedef std::tuple<std::string, Eigen::Vector3d, Eigen::Vector3d> ExternalForce; // body 
    std::map<int, ExternalForce> external_forces;
    external_forces[2]  = std::make_tuple("arm_left_2_link",  Eigen::Vector3d {+0.0, 5.0, 0.0}, Eigen::Vector3d {0.0, 0.0, -0.15});
    external_forces[4]  = std::make_tuple("arm_right_2_link", Eigen::Vector3d {+0.0, 3.0, 0.0}, Eigen::Vector3d {0.0, 0.0, -0.15});
    external_forces[6]  = std::make_tuple("torso_2_link",     Eigen::Vector3d {-5.0, 0.0, 0.0}, Eigen::Vector3d {0.0, 0.0, +0.25});
    external_forces[9] =  std::make_tuple("arm_right_4_link", Eigen::Vector3d {+0.0, 1.0, 0.0}, Eigen::Vector3d {0.0, 0.0, -0.15});
    external_forces[10] = std::make_tuple("arm_left_4_link",  Eigen::Vector3d {+0.0, 9.0, 0.0}, Eigen::Vector3d {0.0, 0.0, -0.15});
    external_forces[12] = std::make_tuple("leg_right_4_link", Eigen::Vector3d {+0.0, 3.0, 0.0}, Eigen::Vector3d {0.0, 0.0, -0.25});
    external_forces[15] = std::make_tuple("leg_left_3_link",  Eigen::Vector3d {-4.0, 1.0, 0.0}, Eigen::Vector3d {0.0, 0.0, -0.25});
    external_forces[17] = std::make_tuple("head_2_link",      Eigen::Vector3d {-9.0, 1.0, 0.0}, Eigen::Vector3d {0.0, 0.0, +0.20});


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

        // update the sensors
        inria_wbc::controllers::SensorData sensor_data;
        // left foot
        sensor_data["lf_torque"] = ft_sensor_left->torque();
        sensor_data["lf_force"] = ft_sensor_left->force();
        // right foot
        sensor_data["rf_torque"] = ft_sensor_right->torque();
        sensor_data["rf_force"] = ft_sensor_right->force();
        // accelerometer
        sensor_data["acceleration"] = imu->linear_acceleration();
        sensor_data["velocity"] = robot->com_velocity().tail<3>();
        // joint positions (excluding floating base)
        sensor_data["positions"] = robot->skeleton()->getPositions().tail(ncontrollable);

        // step the command
        if (simu.schedule(simu.control_freq()))
        {
            auto t1 = high_resolution_clock::now();

            // get actual torque from sensors
            for(auto tq_sens = torque_sensors.cbegin(); tq_sens < torque_sensors.cend(); ++tq_sens)
                tq_sensors(std::distance(torque_sensors.cbegin(), tq_sens)) = (*tq_sens)->torques()(0, 0);


            // safety check on torque sensors (check prevous command with actual sensor measurement)
            if(vm["actuators"].as<std::string>() == "servo" && it_cmd > 0)
            {
                
                if( !torque_collision.check(tsid_tau, tq_sensors, inria_wbc::safety::TorqueCollisionDetection::MovingAverageFilter() ) )
                {
                    std::cerr << "torque discrepancy over threshold: " <<torque_collision.get_discrepancy().maxCoeff() << '\n';
                    auto inv = torque_collision.get_invalid_ids();
                    
/*                     std::cerr <<"invalid dofs: [";
                    for(auto v : inv) std::cerr << v <<" ";
                    std::cerr <<"] with discrepancy [";
                    for(auto v : inv) std::cerr << torque_collision.get_discrepancy()(v) <<" ";
                    std::cerr << "]\n"; */

                    //std::cerr <<"{18, 19, 20, 21} discrepancy: ";
                    //for(auto v : {18, 19, 20, 21}) std::cerr << torque_collision.get_discrepancy()(v) <<" ";
                    //std::cerr <<"\n{14, 15, 16, 17} discrepancy: ";
                    //for(auto v : {14, 15, 16, 17}) std::cerr << torque_collision.get_discrepancy()(v) <<" ";
                    //std::cerr << "\n";

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
                if(external_applied && external_detected) std::cerr << "\033[32mOK\033[37m]";          // green (true positive)
                else if(!external_applied && external_detected) std::cerr << "\033[33mKO\033[37m]";    // yellow (false positive)
                else if(external_applied && !external_detected) std::cerr << "\033[31mKO\033[37m]";    // red (false negative)
                else if(!external_applied && !external_detected) std::cerr << "\033[37mOK\033[37m]";   // white (true negative)
            }

            behavior->update(sensor_data);
            auto q = controller->q(false);
            if (true)
            {
                Eigen::VectorXd cmd;
                if (vm["actuators"].as<std::string>() == "velocity" || vm["actuators"].as<std::string>() == "servo")
                    cmd = inria_wbc::robot_dart::compute_velocities(robot->skeleton(), q, dt);
                else // torque
                    cmd = inria_wbc::robot_dart::compute_spd(robot->skeleton(), q);
                
                //if(external_detected)
                //    cmd.setZero();

                auto cmd_filtered = controller->filter_cmd(cmd).tail(ncontrollable);
                robot->set_commands(cmd_filtered, controllable_dofs);

                // update the expected torque from tsid solution
                tsid_tau = vector_select(controller->tau(), tq_joints_idx);

                //if(external_detected)
                //    tsid_tau = vector_select(robot->gravity_forces(), tq_joints_idx);

            }
            else
            {
                std::cerr << "Solver failed! aborting" << std::endl;
                return -1;
            }
            auto t2 = high_resolution_clock::now();
            time_cmd += duration_cast<microseconds>(t2 - t1).count();
            ++it_cmd;


            // apply scheduled external forces
            auto time_f_ext = external_forces.find(sec);
            if(time_f_ext != external_forces.end())
            {
                auto f_ext = time_f_ext->second;
                robot->set_external_force(std::get<0>(f_ext), std::get<1>(f_ext), std::get<2>(f_ext), true);
                external_applied = true;
                std::cerr << " " << std::get<0>(f_ext);
            }
            else
            {
                robot->clear_external_forces();
                external_applied = false;
            }

            std::cerr << std::endl;
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
