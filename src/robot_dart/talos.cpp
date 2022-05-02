#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <signal.h>

#include <dart/collision/CollisionObject.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
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

#include <boost/program_options.hpp> // Boost need to be always included after pinocchio & inria_wbc

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
        ("actuators,a", po::value<std::string>()->default_value("spd"), "actuator model torque/velocity/servo/spd  [default:spd]")
        ("behavior,b", po::value<std::string>()->default_value("../etc/talos/squat.yaml"), "Configuration file of the tasks (yaml) [default: ../etc/talos/talos_squat.yaml]")
        ("big_window,w", "use a big window (nicer but slower) [default:false]")
        ("check_self_collisions", "check the self collisions (print if a collision)")
        ("check_fall", "check if the robot has fallen (print if a collision)")
        ("collision,k", po::value<std::string>()->default_value("fcl"), "collision engine [default:fcl]")
        ("collisions", po::value<std::string>(), "display the collision shapes for task [name]")
        ("controller,c", po::value<std::string>()->default_value("../etc/talos/talos_pos_tracker.yaml"), "Configuration file of the tasks (yaml) [default: ../etc/talos/talos_pos_tracker.yaml]")
        ("cut", po::value<std::string>()->default_value("leg_left_1_link"), "joint to cut it damage option is enabled")
        ("damage", po::value<bool>()->default_value(false), "damage talos")
        ("duration,d", po::value<int>()->default_value(20), "duration in seconds [20]")
        ("enforce_position,e", po::value<bool>()->default_value(true), "enforce the positions of the URDF [default:true]")
        ("fast,f", "fast (simplified) Talos [default: false]")
        ("control_freq", po::value<int>()->default_value(1000), "set the control frequency")
        ("sim_freq", po::value<int>()->default_value(1000), "set the simulation frequency")
        ("srdf,s", po::value<float>()->default_value(0.0), "save the configuration at the specified time")
        ("ghost,g", "display the ghost (Pinocchio model)")
        ("model_collisions",po::value<bool>()->default_value(false), "display pinocchio qp model collision spheres")
        ("closed_loop", "Close the loop with floating base position and joint positions; required for torque control [default: from YAML file]")
        ("help,h", "produce help message")
        ("height", po::value<bool>()->default_value(false), "print total feet force data to adjust height in config")
        ("mp4,m", po::value<std::string>(), "save the display to a mp4 video [filename]")
        ("push,p", po::value<std::vector<float>>(), "push the robot at t=x1 0.25 s")
        ("norm_force,n", po::value<float>()->default_value(-150) , "push norm force value")
        ("verbose,v", "verbose mode (controller)")
        ("save_traj,S", po::value<std::vector<std::string>>()->multitoken(), "save the trajectory in dir <dir> for references <refs>: -S traj1 rh lh com")
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

        ///// CONTROLLER
        auto controller_path = vm["controller"].as<std::string>();
        auto controller_config = IWBC_CHECK(YAML::LoadFile(controller_path));
        // do some modifications according to command-line options
        controller_config["CONTROLLER"]["base_path"] = "../etc/talos"; // we assume that we run in ./build
        controller_config["CONTROLLER"]["urdf"] = robot->model_filename();
        // controller_config["CONTROLLER"]["urdf"] = "../etc/talos/talos_fast_collisions_d_reflex.urdf";
        controller_config["CONTROLLER"]["mimic_dof_names"] = robot->mimic_dof_names();
        controller_config["CONTROLLER"]["verbose"] = verbose;
        int control_freq = vm["control_freq"].as<int>();
        controller_config["CONTROLLER"]["dt"] = 1.0 / control_freq;
        auto controller_name = IWBC_CHECK(controller_config["CONTROLLER"]["name"].as<std::string>());
        auto closed_loop = IWBC_CHECK(controller_config["CONTROLLER"]["closed_loop"].as<bool>());
        if (vm.count("closed_loop")) {
            closed_loop = true;
            controller_config["CONTROLLER"]["closed_loop"] = true;
        }

        if (vm["actuators"].as<std::string>() == "torque" && !closed_loop)
            std::cout << "WARNING (iwbc): you should activate the closed loop if you are using torque control! (--closed_loop or yaml)" << std::endl;

        auto controller = inria_wbc::controllers::Factory::instance().create(controller_name, controller_config);
        auto controller_pos = std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller);
        IWBC_ASSERT(controller_pos, "we expect a PosTracker here");

        ///// BEHAVIOR
        auto behavior_path = vm["behavior"].as<std::string>();
        auto behavior_config = IWBC_CHECK(YAML::LoadFile(behavior_path));
        auto behavior_name = IWBC_CHECK(behavior_config["BEHAVIOR"]["name"].as<std::string>());
        auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, controller, behavior_config);
        IWBC_ASSERT(behavior, "invalid behavior");

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
        auto ft_sensor_left = simu->add_sensor<robot_dart::sensor::ForceTorque>(robot, "leg_left_6_joint", control_freq, "parent_to_child");
        auto ft_sensor_right = simu->add_sensor<robot_dart::sensor::ForceTorque>(robot, "leg_right_6_joint", control_freq, "parent_to_child");
        robot_dart::sensor::IMUConfig imu_config;
        imu_config.body = robot->body_node("imu_link"); // choose which body the sensor is attached to
        imu_config.frequency = control_freq; // update rate of the sensor
        auto imu = simu->add_sensor<robot_dart::sensor::IMU>(imu_config);

        //////////////////// START SIMULATION //////////////////////////////////////
        simu->set_control_freq(control_freq); // default = 1000 Hz

        std::shared_ptr<robot_dart::Robot> ghost;
        if (vm.count("ghost") || vm.count("collisions")) {
            ghost = robot->clone_ghost();
            ghost->skeleton()->setPosition(4, -1.57);
            ghost->skeleton()->setPosition(5, 1.1);
            simu->add_robot(ghost);
        }

        // self-collision shapes
        std::vector<std::shared_ptr<robot_dart::Robot>> self_collision_spheres;
        if (vm.count("collisions")) {
            auto task_self_collision = controller_pos->task<tsid::tasks::TaskSelfCollision>(vm["collisions"].as<std::string>());
            for (size_t i = 0; i < task_self_collision->avoided_frames_positions().size(); ++i) {
                auto pos = task_self_collision->avoided_frames_positions()[i];
                auto tf = Eigen::Isometry3d(Eigen::Translation3d(pos[0], pos[1], pos[2]));
                double r0 = task_self_collision->avoided_frames_r0s()[i];
                auto sphere = robot_dart::Robot::create_ellipsoid(Eigen::Vector3d(r0 * 2, r0 * 2, r0 * 2), tf, "fixed", 1, Eigen::Vector4d(0, 1, 0, 0.5), "self-collision-" + std::to_string(i));
                sphere->set_color_mode("aspect");
                self_collision_spheres.push_back(sphere);
                simu->add_visual_robot(self_collision_spheres.back());
            }
        }
        std::vector<std::shared_ptr<robot_dart::sensor::Torque>> torque_sensors;

        auto talos_tracker_controller = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracker>(controller);
        for (const auto& joint : talos_tracker_controller->torque_sensor_joints()) {
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
        // to save trajectories
        std::shared_ptr<inria_wbc::trajs::Saver> traj_saver;
        if (!vm["save_traj"].empty()) {
            auto args = vm["save_traj"].as<std::vector<std::string>>();
            auto name = args[0];
            auto refs = std::vector<std::string>(args.begin() + 1, args.end());
            traj_saver = std::make_shared<inria_wbc::trajs::Saver>(controller_pos, args[0], refs);
        }
        // the main loop
        Eigen::VectorXd cmd;
        inria_wbc::controllers::SensorData sensor_data;
        inria_wbc::utils::Timer timer;
        auto active_dofs_controllable = controllable_dofs; // undamaged case
        auto active_dofs = controller->all_dofs(false); // false here: no filter at all
        inria_wbc::robot_dart::RobotDamages robot_damages(robot, simu, active_dofs_controllable, active_dofs);

        Eigen::VectorXd activated_joints = Eigen::VectorXd::Zero(active_dofs.size());

        bool init_model_sphere_collisions = false;
        std::vector<std::shared_ptr<robot_dart::Robot>> spheres;
        bool is_colliding = false;

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

            // get actual torque from sensors
            for (size_t i = 0; i < torque_sensors.size(); ++i)
                if (torque_sensors[i]->active())
                    tq_sensors(i) = torque_sensors[i]->torques()(0, 0);
                else
                    tq_sensors(i) = 0;

            if (vm["height"].as<bool>() && ft_sensor_left->active() && ft_sensor_right->active())
                std::cout << controller->t() << "  floating base height: " << controller->q(false)[2] << " - total feet force: " << ft_sensor_right->force().norm() + ft_sensor_left->force().norm() << std::endl;

            // step the command
            if (simu->schedule(simu->control_freq())) {

                // update the sensors
                // left foot
                if (ft_sensor_left->active()) {
                    sensor_data["lf_torque"] = ft_sensor_left->torque();
                    sensor_data["lf_force"] = ft_sensor_left->force();
                }
                else {
                    sensor_data["lf_torque"] = Eigen::VectorXd::Constant(3, 1e-8);
                    sensor_data["lf_force"] = Eigen::VectorXd::Constant(3, 1e-8);
                }
                // right foot
                if (ft_sensor_right->active()) {
                    sensor_data["rf_torque"] = ft_sensor_right->torque();
                    sensor_data["rf_force"] = ft_sensor_right->force();
                }
                else {
                    sensor_data["rf_torque"] = Eigen::VectorXd::Constant(3, 1e-8);
                    sensor_data["rf_force"] = Eigen::VectorXd::Constant(3, 1e-8);
                }
                // accelerometer

                sensor_data["imu_pos"] = imu->angular_position_vec();
                sensor_data["imu_vel"] = imu->angular_velocity();
                sensor_data["imu_acc"] = imu->linear_acceleration();
                sensor_data["velocity"] = robot->com_velocity().tail<3>();
                // joint positions / velocities (excluding floating base)
                // 0 for joints that are not in active_dofs_controllable
                Eigen::VectorXd positions = Eigen::VectorXd::Zero(controller->controllable_dofs(false).size());
                Eigen::VectorXd velocities = Eigen::VectorXd::Zero(controller->controllable_dofs(false).size());
                for (size_t i = 0; i < controller->controllable_dofs(false).size(); ++i) {
                    auto name = controller->controllable_dofs(false)[i];
                    if (std::count(active_dofs_controllable.begin(), active_dofs_controllable.end(), name) > 0) {
                        positions(i) = robot->positions({name})[0];
                        velocities(i) = robot->velocities({name})[0];
                    }
                }
                sensor_data["positions"] = positions;
                sensor_data["joints_torque"] = tq_sensors;
                sensor_data["joint_velocities"] = velocities;
                // floating base (perfect: no noise in the estimate)
                sensor_data["floating_base_position"] = inria_wbc::robot_dart::floating_base_pos(robot->positions());
                sensor_data["floating_base_velocity"] = inria_wbc::robot_dart::floating_base_vel(robot->velocities());

                timer.begin("solver");
                behavior->update(sensor_data);
                auto q = controller->q(false);
                timer.end("solver");

                Eigen::VectorXd q_no_mimic = controller->filter_cmd(q).tail(ncontrollable); //no fb
                timer.begin("cmd");
                Eigen::VectorXd q_damaged = inria_wbc::robot_dart::filter_cmd(q_no_mimic, controllable_dofs, active_dofs_controllable);

                if (vm["actuators"].as<std::string>() == "velocity" || vm["actuators"].as<std::string>() == "servo")
                    cmd = inria_wbc::robot_dart::compute_velocities(robot, q_damaged, 1. / control_freq, active_dofs_controllable);
                else if (vm["actuators"].as<std::string>() == "spd") {
                    cmd = inria_wbc::robot_dart::compute_spd(robot, q_damaged, 1. / sim_freq, active_dofs_controllable, false);
                }
                else { // torque
                    Eigen::VectorXd cmd_no_mimic = controller->filter_cmd(controller->tau(false)).tail(ncontrollable);
                    cmd = inria_wbc::robot_dart::filter_cmd(cmd_no_mimic, controllable_dofs, active_dofs_controllable);
                }
                timer.end("cmd");

                Eigen::VectorXd translate_ghost = Eigen::VectorXd::Zero(6);
                if (ghost) {
                    translate_ghost(0) -= 1;
                    ghost->set_positions(controller->filter_cmd(controller->q_solver(false)).tail(ncontrollable), controllable_dofs);
                    ghost->set_positions(controller->q_solver(false).head(6) + translate_ghost, floating_base);
                }

                is_colliding = controller->is_model_colliding();
                if (vm["model_collisions"].as<bool>()) {
                    auto spherical_members = controller->collision_check().spherical_members();
                    auto sphere_color = dart::Color::Green(0.5);

                    if (init_model_sphere_collisions == false) {
                        spheres = inria_wbc::robot_dart::create_spherical_members(spherical_members, *simu, sphere_color);
                        init_model_sphere_collisions = true;
                    }
                    else {
                        inria_wbc::robot_dart::update_spherical_members(spherical_members, spheres, sphere_color, is_colliding, controller->collision_check().collision_index(), translate_ghost.head(3));
                    }
                }
            }

            if (simu->schedule(simu->graphics_freq()) && vm.count("collisions")) {
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

                // auto col = simu->world()->getConstraintSolver()->getLastCollisionResult();
                // size_t nc = col.getNumContacts();
                // size_t contact_count = 0;
                // for (size_t i = 0; i < nc; i++) {
                //     auto& ct = col.getContact(i);
                //     auto f1 = ct.collisionObject1->getShapeFrame();
                //     auto f2 = ct.collisionObject2->getShapeFrame();
                //     std::string name1, name2;
                //     if (f1->isShapeNode())
                //         name1 = f1->asShapeNode()->getBodyNodePtr()->getName();
                //     if (f1->isShapeNode())
                //         name2 = f1->asShapeNode()->getBodyNodePtr()->getName();
                //     std::cout << "contact:" << name1<< " -- " << name2 << std::endl;
                // }
            }

            if (traj_saver)
                traj_saver->update();
            // log if needed
            for (auto& x : log_files) {
                if (x.first == "timing")
                    timer.report(*x.second, simu->scheduler().current_time());
                else if (x.first == "cmd")
                    (*x.second) << cmd.transpose() << std::endl;
                else if (x.first == "tau")
                    (*x.second) << controller->tau().transpose() << std::endl;
                else if (x.first == "com") // the real com
                    (*x.second) << robot->com().transpose() << std::endl;
                else if (x.first == "controller_com") // the com according to controller
                    (*x.second) << controller->com().transpose() << std::endl;
                else if (x.first == "objective_value")
                    (*x.second) << controller_pos->objective_value() << std::endl;
                else if (x.first.find("cost_") != std::string::npos) // e.g. cost_com
                    (*x.second) << controller->cost(x.first.substr(strlen("cost_"))) << std::endl;
                else if (x.first == "ft")
                    (*x.second) << ft_sensor_left->torque().transpose() << " " << ft_sensor_left->force().transpose() << " "
                                << ft_sensor_right->torque().transpose() << " " << ft_sensor_right->force().transpose() << std::endl;
                else if (x.first == "force") // the cop according to controller
                    (*x.second) << ft_sensor_left->force().transpose() << " "
                                << controller->lf_force_filtered().transpose() << " "
                                << ft_sensor_right->force().transpose() << " "
                                << controller->rf_force_filtered().transpose() << std::endl;
                else if (x.first == "controller_momentum") // the momentum according to pinocchio
                    (*x.second) << controller->momentum().transpose() << std::endl;
                else if (x.first == "imu") // the momentum according to pinocchio
                    (*x.second) << imu->angular_position_vec().transpose() << " "
                                << imu->angular_velocity().transpose() << " "
                                << imu->linear_acceleration().transpose() << std::endl;
                else if (x.first == "controller_imu") // the momentum according to pinocchio
                    (*x.second) << controller->robot()->framePosition(controller->tsid()->data(), controller->robot()->model().getFrameId("imu_link")).translation().transpose() << " "
                                << controller->robot()->frameVelocityWorldOriented(controller->tsid()->data(), controller->robot()->model().getFrameId("imu_link")).angular().transpose() << " "
                                << controller->robot()->frameAccelerationWorldOriented(controller->tsid()->data(), controller->robot()->model().getFrameId("imu_link")).linear().transpose() << std::endl;
                else if (x.first == "com_vel")
                    (*x.second) << robot->skeleton()->getCOMSpatialVelocity().head(3).transpose() << std::endl;
                else if (x.first == "momentum") {
                    auto bodies = robot->body_names();
                    Eigen::Vector3d angular_momentum = Eigen::Vector3d::Zero();
                    for (auto& b : bodies)
                        angular_momentum += robot->body_node(b)->getAngularMomentum(robot->com());
                    (*x.second) << -angular_momentum.transpose() << std::endl;
                }
                else if (x.first == "cop") { // the cop according to controller
                    if (controller->cop())
                        (*x.second) << controller->cop().value().transpose() << std::endl;
                    else
                        (*x.second) << Eigen::Vector2d::Constant(1000).transpose() << std::endl;
                }
                else if (x.first == "lcop") { // the cop according to controller
                    if (controller->lcop())
                        (*x.second) << controller->lcop().value().transpose() << std::endl;
                    else
                        (*x.second) << Eigen::Vector2d::Constant(1000).transpose() << std::endl;
                }
                else if (x.first == "rcop") { // the cop according to controller
                    if (controller->rcop())
                        (*x.second) << controller->rcop().value().transpose() << std::endl;
                    else
                        (*x.second) << Eigen::Vector2d::Constant(1000).transpose() << std::endl;
                }
                else if (x.first.find("task_") != std::string::npos) // e.g. task_lh
                {
                    auto ref = controller_pos->se3_task(x.first.substr(strlen("task_")))->getReference();
                    (*x.second) << ref.getValue().transpose() << " "
                                << ref.getDerivative().transpose() << " "
                                << ref.getSecondDerivative().transpose() << std::endl;
                }
                else if (robot->body_node(x.first) != nullptr) {
                    pinocchio::SE3 frame;
                    frame.rotation() = robot->body_pose(x.first).rotation();
                    frame.translation() = robot->body_pose(x.first).translation();

                    Eigen::VectorXd vec(12);
                    tsid::math::SE3ToVector(frame, vec);
                    (*x.second) << vec.transpose() << std::endl;
                }
            }
            if (vm.count("srdf")) {
                auto conf = vm["srdf"].as<float>();
                if (controller->t() >= conf && controller->t() < conf + controller->dt())
                    controller->save_configuration("configuration.srdf");
            }
            if (timer.iteration() == 100) {
                std::ostringstream oss;
#ifdef GRAPHIC // to avoid the warning
                oss.precision(3);
                timer.report(oss, simu->scheduler().current_time(), -1, '\n');
                if (is_colliding)
                    oss << "Model is colliding" << std::endl;
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
