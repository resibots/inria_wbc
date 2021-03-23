
#include <algorithm>
#include <boost/program_options.hpp>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <signal.h>

#include <dart/dynamics/BodyNode.hpp>

#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/sensor/force_torque.hpp>
#include <robot_dart/sensor/imu.hpp>
#include <robot_dart/sensor/torque.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

#include "inria_wbc/behaviors/behavior.hpp"
#include "inria_wbc/controllers/pos_tracker.hpp"
#include "inria_wbc/controllers/talos_pos_tracker.hpp"
#include "inria_wbc/exceptions.hpp"
#include "inria_wbc/robot_dart/cmd.hpp"
#include "inria_wbc/robot_dart/utils.hpp"
#include "inria_wbc/robot_dart/external_collision_detector.hpp"
#include "inria_wbc/robot_dart/self_collision_detector.hpp"
#include "tsid/tasks/task-self-collision.hpp"

static const std::string red = "\x1B[31m";
static const std::string rst = "\x1B[0m";
static const std::string bold = "\x1B[1m";


int main(int argc, char* argv[])
{
    try {
        // program options
        namespace po = boost::program_options;
        po::options_description desc("Test_controller options");
        // clang-format off
        desc.add_options()
        ("actuators,a", po::value<std::string>()->default_value("torque"), "actuator model torque/velocity/servo (always for position control) [default:torque]")
        ("big_window,b", "use a big window (nicer but slower) [default:true]")
        ("check_self_collisions", "check the self collisions (print if a collision)")
        ("check_fall", "check if the robot has fallen (print if a collision)")
        ("collision,k", po::value<std::string>()->default_value("fcl"), "collision engine [default:fcl]")
        ("collisions", po::value<std::string>(), "display the collision shapes for task [name]")
        ("conf,c", po::value<std::string>()->default_value("../etc/squat.yaml"), "Configuration file of the tasks (yaml) [default: ../etc/squat.yaml]")
        ("duration,d", po::value<int>()->default_value(20), "duration in seconds [20]")
        ("enforce_position,e", po::value<bool>()->default_value(true), "enforce the positions of the URDF [default:true]")
        ("fast,f", "fast (simplified) Talos [default: false]")
        ("control_freq", po::value<int>()->default_value(1000), "set the control frequency")
        ("sim_freq", po::value<int>()->default_value(1000), "set the simulation frequency")
        ("ghost,g", "display the ghost (Pinocchio model)")
        ("help,h", "produce help message")
        ("mp4,m", po::value<std::string>(), "save the display to a mp4 video [filename]")
        ("push,p", po::value<std::vector<float>>(), "push the robot at t=x1 0.25 s")
        ("verbose,v", "verbose mode (controller)")
        ("log,l", po::value<std::vector<std::string>>()->default_value(std::vector<std::string>(),""), 
            "log the trajectory of a dart body [with urdf names] or timing or CoM or cost, example: -l timing -l com -l lf -l cost_com -l cost_lf")
        ;
        // clang-format on
        po::variables_map vm;
        try {
            po::store(po::parse_command_line(argc, argv, desc), vm);
            po::notify(vm);
        }
        catch (po::too_many_positional_options_error& e) {
            // A positional argument like `opt2=option_value_2` was given
            std::cerr << e.what() << std::endl;
            std::cerr << desc << std::endl;
            return 1;
        }
        catch (po::error_with_option_name& e) {
            // Another usage error occurred
            std::cerr << e.what() << std::endl;
            std::cerr << desc << std::endl;
            return 1;
        }

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }

        // clang-format off
    std::cout<< "------ CONFIGURATION ------" << std::endl;
    std::ostringstream oss_conf;
    for (const auto& kv : vm){
        oss_conf << kv.first << " ";
        try { oss_conf << kv.second.as<std::string>();
        } catch(...) {/* do nothing */ }
        try { oss_conf << kv.second.as<bool>();
        } catch(...) {/* do nothing */ }
        try { oss_conf << kv.second.as<int>();
        } catch(...) {/* do nothing */ }
        oss_conf << std::endl;
    }
    std::cout << oss_conf.str();
    std::cout << "--------------------------" << std::endl;
        // clang-format on

        bool verbose = (vm.count("verbose") != 0);
        std::map<std::string, std::shared_ptr<std::ofstream>> log_files;
        for (auto& x : vm["log"].as<std::vector<std::string>>())
            log_files[x] = std::make_shared<std::ofstream>((x + ".dat").c_str());

        // dt of the simulation and the controller
        int sim_freq = vm["sim_freq"].as<int>();
        float dt = 1.0f/sim_freq;
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
        if (vm.count("big_window")) {
            configuration.width = 1280;
            configuration.height = 960;
        }
        else {
            configuration.width = 800;
            configuration.height = 500;
        }

        auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(configuration);
        simu.set_graphics(graphics);
        graphics->look_at({3.5, -2, 2.2}, {0., 0., 1.4});
        if (vm.count("mp4"))
            graphics->record_video(vm["mp4"].as<std::string>());
#endif
        simu.add_robot(robot);
        auto floor = simu.add_checkerboard_floor();

        //////////////////// INIT STACK OF TASK //////////////////////////////////////
        std::string sot_config_path = vm["conf"].as<std::string>();
        int control_freq = vm["control_freq"].as<int>();
        inria_wbc::controllers::Controller::Params params = {
            robot->model_filename(),
            sot_config_path,
            1.0f / control_freq,
            verbose,
            robot->mimic_dof_names()};

        YAML::Node config = IWBC_CHECK(YAML::LoadFile(sot_config_path));

        auto controller_name = IWBC_CHECK(config["CONTROLLER"]["name"].as<std::string>());
        auto controller = inria_wbc::controllers::Factory::instance().create(controller_name, params);
        auto controller_pos = std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller);
        IWBC_ASSERT(controller, "we expect a PosTracker here");

        auto behavior_name = IWBC_CHECK(config["BEHAVIOR"]["name"].as<std::string>());
        auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, controller);
        assert(behavior);


        auto all_dofs = controller->all_dofs();
        auto floating_base = all_dofs;
        floating_base.resize(6);

        auto controllable_dofs = controller->controllable_dofs();
        robot->set_positions(controller->q0(), all_dofs);

        uint ncontrollable = controllable_dofs.size();

        if (vm.count("log")) {
            std::ofstream ofs("all_dofs.dat");
            for (auto& c : all_dofs)
                ofs << c << std::endl;
        }

        // add sensors to the robot
        auto ft_sensor_left = simu.add_sensor<robot_dart::sensor::ForceTorque>(robot, "leg_left_6_joint");
        auto ft_sensor_right = simu.add_sensor<robot_dart::sensor::ForceTorque>(robot, "leg_right_6_joint");
        robot_dart::sensor::IMUConfig imu_config;
        imu_config.body = robot->body_node("imu_link"); // choose which body the sensor is attached to
        imu_config.frequency = control_freq; // update rate of the sensor
        auto imu = simu.add_sensor<robot_dart::sensor::IMU>(imu_config);

        //////////////////// START SIMULATION //////////////////////////////////////
        simu.set_control_freq(control_freq); // default = 1000 Hz
        double time_simu = 0, time_cmd = 0, time_solver = 0, max_time_solver = 0, min_time_solver = 1e10;       
        int it_simu = 0, it_cmd = 0;

        std::shared_ptr<robot_dart::Robot> ghost;
        if (vm.count("ghost") || vm.count("collisions")) {
            ghost = robot->clone_ghost();
            ghost->skeleton()->setPosition(4, -1.57);
            ghost->skeleton()->setPosition(5, 1.1);
            simu.add_robot(ghost);
        }

        // self-collision shapes
        std::vector<std::shared_ptr<robot_dart::Robot>> self_collision_spheres;
        if (vm.count("collisions")) {
            auto task_self_collision = controller_pos->task<tsid::tasks::TaskSelfCollision>(vm["collisions"].as<std::string>());
            for (size_t i = 0; i < task_self_collision->avoided_frames_positions().size(); ++i) {
                auto pos = task_self_collision->avoided_frames_positions()[i];
                auto tf =  Eigen::Isometry3d(Eigen::Translation3d(pos[0], pos[1], pos[2]));
                double r0 = task_self_collision->avoided_frames_r0s()[i];
                auto sphere = robot_dart::Robot::create_ellipsoid(Eigen::Vector3d(r0 * 2, r0 * 2, r0 * 2), tf, "fixed", 1, Eigen::Vector4d(0, 1, 0, 0.5), "self-collision-" + std::to_string(i));
                sphere->set_color_mode("aspect");
                self_collision_spheres.push_back(sphere);
                simu.add_visual_robot(self_collision_spheres.back());
            }
        }
        std::vector<std::shared_ptr<robot_dart::sensor::Torque>> torque_sensors;

        auto talos_tracker_controller = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracker>(controller);
        for (const auto& joint : talos_tracker_controller->torque_sensor_joints()) {
            torque_sensors.push_back(simu.add_sensor<robot_dart::sensor::Torque>(robot, joint, 1000));
            std::cerr << "Add joint torque sensor:  " << joint << std::endl;
        }

        // reading from sensors
        Eigen::VectorXd tq_sensors = Eigen::VectorXd::Zero(torque_sensors.size());

        // create the collision detectors (useful only if --check_self_collisions)
        inria_wbc::robot_dart::SelfCollisionDetector collision_detector(robot);
        std::map<std::string, std::string> filter_body_names_pairs;
        filter_body_names_pairs["leg_right_6_link"] = "BodyNode";
        filter_body_names_pairs["leg_left_6_link"] = "BodyNode";
        inria_wbc::robot_dart::ExternalCollisionDetector floor_collision_detector(robot, floor, filter_body_names_pairs);

        // the main loop
        using namespace std::chrono;
        while (simu.scheduler().next_time() < vm["duration"].as<int>() && !simu.graphics()->done()) {
            double time_step_solver = 0, time_step_cmd = 0, time_step_simu = 0;

            if (vm.count("check_self_collisions")) {

                IWBC_ASSERT(!vm.count("fast"), "=> check_self_collisions is not compatible with --fast!");
                auto collision_list = collision_detector.collide();
                if (!collision_list.empty())
                    std::cout << " ------ SELF Collisions ------ " << std::endl;
                for (auto& s : collision_list)
                    std::cout << s << std::endl;
            }

            if (vm.count("check_fall")) {
                auto head_z_diff = std::abs(controller->model_frame_pos("head_1_link").translation()(2) - robot->body_pose("head_1_link").translation()(2));
                std::vector<std::string> floor_collision_list;
                if (head_z_diff > 0.75)
                    floor_collision_list.push_back("head_1_link");
                if (!vm.count("fast")) {
                    auto list2 = floor_collision_detector.collide();
                    floor_collision_list.insert(floor_collision_list.end(), list2.begin(), list2.end());
                }
                if (!floor_collision_list.empty())
                    std::cout << " ------ FLOOR Collisions ------ " << std::endl;
                for (auto& s : floor_collision_list)
                    std::cout << s << std::endl;
            }

            // step the command
            Eigen::VectorXd cmd;
            if (simu.schedule(simu.control_freq())) {
                 // get actual torque from sensors
                for (auto tq_sens = torque_sensors.cbegin(); tq_sens < torque_sensors.cend(); ++tq_sens)
                    tq_sensors(std::distance(torque_sensors.cbegin(), tq_sens)) = (*tq_sens)->torques()(0, 0);

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
                sensor_data["positions"] = robot->positions(controllable_dofs);
                // joint torque sensors
                sensor_data["joints_torque"] = tq_sensors;
                sensor_data["joint_velocities"] = robot->velocities(controllable_dofs);
                // floating base (perfect: no noise in the estimate)
                sensor_data["floating_base_position"] = inria_wbc::robot_dart::floating_base_pos(robot->positions());
                sensor_data["floating_base_velocity"] = inria_wbc::robot_dart::floating_base_vel(robot->velocities());

                auto t1_solver = high_resolution_clock::now();
                behavior->update(sensor_data);
                auto q = controller->q(false);
                auto t2_solver = high_resolution_clock::now();
                time_step_solver = duration_cast<microseconds>(t2_solver - t1_solver).count();

                auto t1_cmd = high_resolution_clock::now();
                if (vm["actuators"].as<std::string>() == "velocity" || vm["actuators"].as<std::string>() == "servo")
                    cmd = inria_wbc::robot_dart::compute_velocities(robot->skeleton(), q, 1./control_freq);
                else // torque
                    cmd = controller->tau();
                    //inria_wbc::robot_dart::compute_spd(robot->skeleton(), q, 1./control_freq);
                auto t2_cmd = high_resolution_clock::now();
                time_step_cmd = duration_cast<microseconds>(t2_cmd - t1_cmd).count();

                robot->set_commands(controller->filter_cmd(cmd).tail(ncontrollable), controllable_dofs);
                if (ghost) {
                    Eigen::VectorXd translate_ghost = Eigen::VectorXd::Zero(6);
                    translate_ghost(0) -= 1;
                    ghost->set_positions(controller->filter_cmd(q).tail(ncontrollable), controllable_dofs);
                    ghost->set_positions(q.head(6) + translate_ghost, floating_base);
                }

                ++it_cmd;
                max_time_solver = std::max(time_step_solver, max_time_solver);
                min_time_solver = std::min(time_step_solver, min_time_solver);
            }

            if (simu.schedule(simu.graphics_freq()) && vm.count("collisions")) {
                auto controller_pos = std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller);
                auto task_self_collision = controller_pos->task<tsid::tasks::TaskSelfCollision>(vm["collisions"].as<std::string>());
                for (size_t i = 0; i < task_self_collision->avoided_frames_positions().size(); ++i) {
                    auto cp = self_collision_spheres[i]->base_pose();
                    cp.translation() = task_self_collision->avoided_frames_positions()[i];
                    cp.translation()[0] -= 1; // move to the ghost
                    self_collision_spheres[i]->set_base_pose(cp);
                    auto bd = self_collision_spheres[i]->skeleton()->getBodyNodes()[0];
                    auto visual = bd->getShapeNodesWith<dart::dynamics::VisualAspect>()[0];
                    visual->getShape()->setDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR);
                    bool c = task_self_collision->collision(i);
                    if (c) {
                        visual->getVisualAspect()->setRGBA(dart::Color::Red(1.0));
                    }
                    else {
                        visual->getVisualAspect()->setRGBA(dart::Color::Green(1.0));
                    }
                }
            }

            // push the robot
            bool push = false;
            if (vm.count("push")) {
                auto pv = vm["push"].as<std::vector<float>>();
                for (auto& p : pv) {
                    if (simu.scheduler().current_time() > p && simu.scheduler().current_time() < p + 0.5) {
                        robot->set_external_force("base_link", Eigen::Vector3d(-150, 0, 0));
                        push = true;
                    }
                    if (simu.scheduler().current_time() > p + 0.25)
                        robot->clear_external_forces();
                }
            }

            // step the simulation
            {
                auto t1_simu = high_resolution_clock::now();
                simu.step_world();
                auto t2_simu = high_resolution_clock::now();
                time_step_simu = duration_cast<microseconds>(t2_simu - t1_simu).count();
                ++it_simu;
            }

            // log if needed
            for (auto& x : log_files) {
                if (x.first == "timing")
                    (*x.second) << time_step_solver / 1000.0 << "\t" << time_step_cmd / 1000.0 << "\t" << time_step_simu / 1000.0 << std::endl;
                else if (x.first == "cmd")
                    (*x.second) << cmd.transpose() << std::endl;
                else if (x.first == "com") // the real com
                    (*x.second) << robot->com().transpose() << std::endl;
                else if (x.first == "controller_com") // the com according to controller
                    (*x.second) << controller->com().transpose() << std::endl;
                else if (x.first == "cop") // the cop according to controller
                    (*x.second) << controller->cop().transpose() << std::endl;
                else if (x.first.find("cost_") != std::string::npos) // e.g. cost_com
                    (*x.second) << controller->cost(x.first.substr(strlen("cost_"))) << std::endl;
                else if (x.first == "ft")
                    (*x.second) << ft_sensor_left->torque().transpose() << " " << ft_sensor_left->force().transpose() << " "
                                << ft_sensor_right->torque().transpose() << " " << ft_sensor_right->force().transpose() << std::endl;
                else if (x.first == "momentum") // the momentum according to pinocchio
                    (*x.second) << controller->momentum().transpose() << std::endl;
                else if (x.first == "ref_com")
                    (*x.second) << controller_pos->get_com_ref().transpose() << std::endl;
                else if (x.first.find("ref_") != std::string::npos) // e.g. tsid_lh (lh = task name)
                    (*x.second) << controller_pos->get_se3_ref(x.first.substr(strlen("ref_"))).translation().transpose() << std::endl;
                else
                    (*x.second) << robot->body_pose(x.first).translation().transpose() << std::endl;
            }
            // print timing information
            time_simu += time_step_simu;
            time_cmd += time_step_cmd;
            time_solver += time_step_solver;

            if (it_simu == 100) {
                double t_sim = time_simu / it_simu / 1000.;
                double t_cmd = time_cmd / it_cmd / 1000.;
                double t_solver = time_solver / it_cmd / 1000.;

                std::cout << "t=" << simu.scheduler().current_time()
                          << "\tit. simu: " << t_sim << " ms"
                          << "\tit. solver:" << t_solver << " ms [" << min_time_solver / 1000. << " " << max_time_solver / 1000. << "]"
                          << "\tit. cmd:" << t_cmd << " ms"
                          << std::endl;
                std::ostringstream oss;
                oss.precision(3);
                oss << "[Sim: " << t_sim << " ms]" << std::endl;
                oss << "[Solver: " << t_solver << " ms]" << std::endl;
                oss << "[Cmd: " << t_cmd << " ms]" << std::endl;
                if (push)
                    oss << "pushing..." << std::endl;
#ifdef GRAPHIC // to avoid the warning
                if (!vm.count("mp4"))
                    simu.set_text_panel(oss.str());
#endif
                it_simu = 0;
                it_cmd = 0;
                time_cmd = 0;
                time_simu = 0;
                time_solver = 0;
                min_time_solver = 1e10;
                max_time_solver = 0;
            }
        }
    }
    catch (YAML::RepresentationException& e) {
        std::cout << red << bold << "YAML Parse error (missing key in YAML file?): " << rst << e.what() << std::endl;
    }
    catch (YAML::ParserException& e) {
        std::cout << red << bold << "YAML Parse error: " << rst << e.what() << std::endl;
    }
    catch (std::exception& e) {
        std::cout << red << bold << "Error (exception): " << rst << e.what() << std::endl;
    }
    return 0;
}
