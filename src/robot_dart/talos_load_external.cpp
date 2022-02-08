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
#include "inria_wbc/robot_dart/damages.hpp"
#include "inria_wbc/robot_dart/external_collision_detector.hpp"
#include "inria_wbc/robot_dart/self_collision_detector.hpp"
#include "inria_wbc/robot_dart/utils.hpp"
#include "inria_wbc/trajs/loader.hpp"
#include "inria_wbc/trajs/saver.hpp"
#include "inria_wbc/utils/timer.hpp"
#include "tsid/tasks/task-self-collision.hpp"

static const std::string red = "\x1B[31m";
static const std::string rst = "\x1B[0m";
static const std::string bold = "\x1B[1m";

// This loads an external joint position command trajectory from the --load_external csv file
// Those commands are sent directly to robot_dart without any controller/behavior/qp
int main(int argc, char* argv[])
{
    try {
        // program options
        namespace po = boost::program_options;
        po::options_description desc("Test_controller options");
        // clang-format off
        desc.add_options()
        ("actuators,a", po::value<std::string>()->default_value("spd"), "actuator model velocity/servo/spd  [default:spd]")
        ("big_window,w", "use a big window (nicer but slower) [default:false]")
        ("check_self_collisions", "check the self collisions (print if a collision)")
        ("check_fall", "check if the robot has fallen (print if a collision)")
        ("collision,k", po::value<std::string>()->default_value("fcl"), "collision engine [default:fcl]")
        ("cut", po::value<std::string>()->default_value("leg_left_1_link"), "joint to cut it damage option is enabled")
        ("damage", po::value<bool>()->default_value(false), "damage talos")
        ("duration,d", po::value<int>()->default_value(20), "duration in seconds [20]")
        ("enforce_position,e", po::value<bool>()->default_value(true), "enforce the positions of the URDF [default:true]")
        ("fast,f", "fast (simplified) Talos [default: false]")
        ("control_freq", po::value<int>()->default_value(500), "set the control frequency")
        ("sim_freq", po::value<int>()->default_value(500), "set the simulation frequency")
        ("help,h", "produce help message")
        ("mp4,m", po::value<std::string>(), "save the display to a mp4 video [filename]")
        ("push,p", po::value<std::vector<float>>(), "push the robot at t=x1 0.25 s")
        ("norm_force,n", po::value<float>()->default_value(-150) , "push norm force value")
        ("log,l", po::value<std::vector<std::string>>()->default_value(std::vector<std::string>(),""), 
            "log the trajectory of a dart body [with urdf names] or timing or CoM or cost, example: -l timing -l com -l lf -l cost_com -l cost_lf")
        ("load_external, L", po::value<std::string>()->default_value("../etc/talos/load_external/trajectory.yaml"), "load external joint position commands")
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

        std::map<std::string, std::shared_ptr<std::ofstream>> log_files;
        for (auto& x : vm["log"].as<std::vector<std::string>>())
            log_files[x] = std::make_shared<std::ofstream>((x + ".dat").c_str());

        // dt of the simulation and the controller
        int sim_freq = vm["sim_freq"].as<int>();
        float dt = 1.0f / sim_freq;
        std::cout << "dt:" << dt << std::endl;

        //////////////////// INIT DART ROBOT //////////////////////////////////////
        std::srand(std::time(NULL));
        // std::vector<std::pair<std::string, std::string>> packages = {{"talos_data", "/home/pal/talos_data"}};
        // std::string urdf = vm.count("fast") ? "talos/talos_fast.urdf" : "talos/talos.urdf";
        // urdf = "/home/pal/talos_data/example-robot-data/robots/talos_data/robots/talos_full_v2.urdf";
        std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
        std::string urdf = vm.count("fast") ? "talos/talos_fast.urdf" : "talos/talos.urdf";
        auto robot = std::make_shared<robot_dart::Robot>(urdf, packages);
        robot->set_position_enforced(vm["enforce_position"].as<bool>());
        if (vm["actuators"].as<std::string>() == "spd")
            robot->set_actuator_types("torque");
        else
            robot->set_actuator_types(vm["actuators"].as<std::string>());

        int control_freq = vm["control_freq"].as<int>();

        //////////////////// INIT DART SIMULATION WORLD //////////////////////////////////////
        auto simu = std::make_shared<robot_dart::RobotDARTSimu>(dt);
        simu->set_collision_detector(vm["collision"].as<std::string>());

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
        simu->set_graphics(graphics);
        graphics->look_at({3.5, -2, 2.2}, {0., 0., 1.4});
        if (vm.count("mp4"))
            graphics->record_video(vm["mp4"].as<std::string>());
#endif
        simu->add_robot(robot);
        auto floor = simu->add_checkerboard_floor();

        // add sensors to the robot
        auto ft_sensor_left = simu->add_sensor<robot_dart::sensor::ForceTorque>(robot, "leg_left_6_joint", control_freq, "parent_to_child");
        auto ft_sensor_right = simu->add_sensor<robot_dart::sensor::ForceTorque>(robot, "leg_right_6_joint", control_freq, "parent_to_child");
        robot_dart::sensor::IMUConfig imu_config;
        imu_config.body = robot->body_node("imu_link"); // choose which body the sensor is attached to
        imu_config.frequency = control_freq; // update rate of the sensor
        auto imu = simu->add_sensor<robot_dart::sensor::IMU>(imu_config);

        //////////////////// START SIMULATION //////////////////////////////////////
        simu->set_control_freq(control_freq); // default = 1000 Hz

        std::vector<std::shared_ptr<robot_dart::sensor::Torque>> torque_sensors;
        std::vector<std::string> torque_collision_joints = {
            "leg_left_1_joint", "leg_left_2_joint", "leg_left_3_joint", "leg_left_4_joint", "leg_left_5_joint", "leg_left_6_joint",
            "leg_right_1_joint", "leg_right_2_joint", "leg_right_3_joint", "leg_right_4_joint", "leg_right_5_joint", "leg_right_6_joint",
            "torso_1_joint", "torso_2_joint",
            "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint",
            "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint"};

        for (const auto& joint : torque_collision_joints) {
            torque_sensors.push_back(simu->add_sensor<robot_dart::sensor::Torque>(robot, joint, control_freq));
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

        using namespace std::chrono;

        // the main loop
        Eigen::VectorXd cmd;
        inria_wbc::controllers::SensorData sensor_data;
        inria_wbc::utils::Timer timer;
        auto all_dofs = robot->dof_names();
        auto controllable_dofs_fb = robot->dof_names(true, true, true);
        std::vector<std::string> fb_dofs(controllable_dofs_fb.begin(), controllable_dofs_fb.begin() + 6); // supress fb dofs
        std::vector<std::string> active_dofs_controllable(controllable_dofs_fb.begin() + 6, controllable_dofs_fb.end()); // supress fb dofs
        auto active_dofs = robot->dof_names(); // false here: no filter at all
        inria_wbc::robot_dart::RobotDamages robot_damages(robot, simu, active_dofs_controllable, active_dofs);
        Eigen::VectorXd activated_joints = Eigen::VectorXd::Zero(active_dofs.size());

        std::shared_ptr<inria_wbc::trajs::Loader> traj_loader;
        int it_loader = 0;
        int it_loader_end = 0;
        auto traj_path = vm["load_external"].as<std::string>();
        traj_loader = std::make_shared<inria_wbc::trajs::Loader>(traj_path);
        auto rnv = traj_loader->ref_names_vec();
        IWBC_ASSERT(std::find(rnv.begin(), rnv.end(), "q") != rnv.end(), "You need to give a posture to the trajectory loader");
        it_loader_end = traj_loader->size_vec();
        IWBC_ASSERT(it_loader_end > 0, "Your posture trajectory does not contain enough points");

        bool begin = true;

        while (simu->scheduler().next_time() < vm["duration"].as<int>() && !simu->graphics()->done()) {
            if (vm["damage"].as<bool>()) {
                try {

                    if (simu->scheduler().current_time() == 0.0) {
                        collision_detector.remove_frames();
                        robot_damages.cut(vm["cut"].as<std::string>());
                        collision_detector.add_frames();
                        active_dofs_controllable = robot_damages.active_dofs_controllable();
                        active_dofs = robot_damages.active_dofs();
                    }
                }
                catch (std::exception& e) {
                    std::cout << red << bold << "Error (exception): " << rst << e.what() << std::endl;
                }
            }

            if (vm.count("check_self_collisions")) {
                IWBC_ASSERT(!vm.count("fast"), "=> check_self_collisions is not compatible with --fast!");
                auto collision_list = collision_detector.collide();
                if (!collision_list.empty())
                    std::cout << " ------ SELF Collisions ------ " << std::endl;
                for (auto& s : collision_list)
                    std::cout << s << std::endl;
            }

            if (vm.count("check_fall")) {
                auto head_z_diff = std::abs(robot->body_pose("head_1_link").translation()(2));
                std::vector<std::string> floor_collision_list;
                if (head_z_diff < 0.40)
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

            // get actual torque from sensors
            for (size_t i = 0; i < torque_sensors.size(); ++i)
                if (torque_sensors[i]->active())
                    tq_sensors(i) = torque_sensors[i]->torques()(0, 0);
                else
                    tq_sensors(i) = 0;

            // step the command
            if (simu->schedule(simu->control_freq())) {

                timer.begin("cmd");
                IWBC_ASSERT((traj_loader->task_ref_vec("q", it_loader).size() - 7) == active_dofs_controllable.size(), "Loader: your recording should match active_dofs_controllable ", traj_loader->task_ref_vec("q", it_loader).size() - 7, "!=", active_dofs_controllable.size());
                Eigen::VectorXd q_loader = traj_loader->task_ref_vec("q", it_loader);

                if (begin) {
                    Eigen::VectorXd q_dart = Eigen::VectorXd::Zero(q_loader.size() - 1);
                    Eigen::Quaterniond quat(q_loader(6), q_loader(3), q_loader(4), q_loader(5));
                    Eigen::AngleAxisd aaxis(quat);
                    q_dart << aaxis.angle() * aaxis.axis(), q_loader.head(3), q_loader.tail(q_loader.size() - 7);
                    robot->set_positions(q_dart.tail(q_dart.size() - 6), active_dofs_controllable);
                    robot->set_positions(q_dart.head(6), fb_dofs);
                    begin = false;
                }
                Eigen::VectorXd q_damaged = Eigen::VectorXd::Zero(q_loader.size() - 7);
                q_damaged = q_loader.tail(q_damaged.size());
                if (it_loader < it_loader_end - 1)
                    it_loader++;

                if (vm["actuators"].as<std::string>() == "velocity" || vm["actuators"].as<std::string>() == "servo")
                    cmd = inria_wbc::robot_dart::compute_velocities(robot, q_damaged, 1. / control_freq, active_dofs_controllable);
                else if (vm["actuators"].as<std::string>() == "spd") {
                    cmd = inria_wbc::robot_dart::compute_spd(robot, q_damaged, 1. / sim_freq, active_dofs_controllable, false);
                }

                timer.end("cmd");
            }

            // push the robot
            bool push = false;
            if (vm.count("push")) {
                auto pv = vm["push"].as<std::vector<float>>();
                auto pforce = vm["norm_force"].as<float>();
                for (auto& p : pv) {
                    if (simu->scheduler().current_time() > p && simu->scheduler().current_time() < p + 0.5) {
                        robot->set_external_force("base_link", Eigen::Vector3d(0, pforce, 0));
                        // robot->set_external_force("base_link", Eigen::Vector3d(pforce, 0, 0));
                        push = true;
                    }
                    if (simu->scheduler().current_time() > p + 0.25)
                        robot->clear_external_forces();
                }
            }

            // step the simulation
            {
                timer.begin("sim");
                robot->set_commands(cmd, active_dofs_controllable);
                simu->step_world();
                timer.end("sim");
            }

            // log if needed
            for (auto& x : log_files) {
                if (x.first == "timing")
                    timer.report(*x.second, simu->scheduler().current_time());
                else if (x.first == "cmd")
                    (*x.second) << cmd.transpose() << std::endl;
                else if (x.first == "com") // the real com
                    (*x.second) << robot->com().transpose() << std::endl;
                else if (x.first == "ft")
                    (*x.second) << ft_sensor_left->torque().transpose() << " " << ft_sensor_left->force().transpose() << " "
                                << ft_sensor_right->torque().transpose() << " " << ft_sensor_right->force().transpose() << std::endl;
                else if (x.first == "force") // the cop according to controller
                    (*x.second) << ft_sensor_left->force().transpose() << " "
                                << ft_sensor_right->force().transpose() << " " << std::endl;
                else
                    (*x.second) << robot->body_pose(x.first).translation().transpose() << std::endl;
            }

            if (timer.iteration() == 100) {
                std::ostringstream oss;
#ifdef GRAPHIC // to avoid the warning
                oss.precision(3);
                timer.report(oss, simu->scheduler().current_time(), -1, '\n');
                if (!vm.count("mp4"))
                    simu->set_text_panel(oss.str());
#endif
            }
            timer.report(simu->scheduler().current_time(), 100);
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
