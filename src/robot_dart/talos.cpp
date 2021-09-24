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
#include "inria_wbc/trajs/saver.hpp"
#include "inria_wbc/exceptions.hpp"
#include "inria_wbc/robot_dart/cmd.hpp"
#include "inria_wbc/robot_dart/external_collision_detector.hpp"
#include "inria_wbc/robot_dart/self_collision_detector.hpp"
#include "inria_wbc/robot_dart/utils.hpp"
#include "tsid/tasks/task-self-collision.hpp"
#include "inria_wbc/utils/timer.hpp"

static const std::string red = "\x1B[31m";
static const std::string rst = "\x1B[0m";
static const std::string bold = "\x1B[1m";

// make a command with the subset of new_joints (e.g., for damage)
Eigen::VectorXd filter_cmd(const Eigen::VectorXd& cmds,
    const std::vector<std::string>& orig_joints,
    const std::vector<std::string>& new_joints)
{
    Eigen::VectorXd new_commands = Eigen::VectorXd::Zero(new_joints.size());
    for (size_t n = 0; n < new_joints.size(); ++n)
        for (size_t k = 0; k < orig_joints.size(); ++k)
            if (new_joints[n] == orig_joints[k])
                new_commands(n) = cmds(k);
    return new_commands;
}


// remove some joints from the command (e.g., mimic, damage, etc.)
// TODO : optimize with indices computed once, then use slice
Eigen::VectorXd rm_from_command(const Eigen::VectorXd& cmds,
    const std::vector<std::string>& orig_joints,
    const std::vector<std::string>& to_remove)
{
    Eigen::VectorXd new_commands = Eigen::VectorXd::Zero(cmds.size());
    int j = 0;
    for (size_t n = 0; n < orig_joints.size(); ++n)
        if (std::find(to_remove.begin(), to_remove.end(), orig_joints[n]) == to_remove.end())
            new_commands(j++) = cmds(n);
    return new_commands.head(j);
}

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
        ("duration,d", po::value<int>()->default_value(20), "duration in seconds [20]")
        ("enforce_position,e", po::value<bool>()->default_value(true), "enforce the positions of the URDF [default:true]")
        ("fast,f", "fast (simplified) Talos [default: false]")
        ("control_freq", po::value<int>()->default_value(1000), "set the control frequency")
        ("sim_freq", po::value<int>()->default_value(1000), "set the simulation frequency")
        ("srdf,s", po::value<float>()->default_value(0.0), "save the configuration at the specified time")
        ("ghost,g", "display the ghost (Pinocchio model)")
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
        std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
        std::string urdf = vm.count("fast") ? "talos/talos_fast.urdf" : "talos/talos.urdf";
        auto robot = std::make_shared<robot_dart::Robot>(urdf, packages);
        robot->set_position_enforced(vm["enforce_position"].as<bool>());
        if (vm["actuators"].as<std::string>() == "spd")
            robot->set_actuator_types("torque");
        else
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

        ///// CONTROLLER
        auto controller_path = vm["controller"].as<std::string>();
        auto controller_config = IWBC_CHECK(YAML::LoadFile(controller_path));
        // do some modifications according to command-line options
        controller_config["CONTROLLER"]["base_path"] = "../etc/talos"; // we assume that we run in ./build
        controller_config["CONTROLLER"]["urdf"] = robot->model_filename();
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
        auto ft_sensor_left = simu.add_sensor<robot_dart::sensor::ForceTorque>(robot, "leg_left_6_joint", control_freq);
        auto ft_sensor_right = simu.add_sensor<robot_dart::sensor::ForceTorque>(robot, "leg_right_6_joint", control_freq);
        robot_dart::sensor::IMUConfig imu_config;
        imu_config.body = robot->body_node("imu_link"); // choose which body the sensor is attached to
        imu_config.frequency = control_freq; // update rate of the sensor
        auto imu = simu.add_sensor<robot_dart::sensor::IMU>(imu_config);

        //////////////////// START SIMULATION //////////////////////////////////////
        simu.set_control_freq(control_freq); // default = 1000 Hz

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
                auto tf = Eigen::Isometry3d(Eigen::Translation3d(pos[0], pos[1], pos[2]));
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
            torque_sensors.push_back(simu.add_sensor<robot_dart::sensor::Torque>(robot, joint, control_freq));
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
        bool damaged = false;
        inria_wbc::controllers::SensorData sensor_data;
        auto dart_joint_names = controllable_dofs; // undamaged case
        auto dart_all_dofs = controller->all_dofs(false); // false here: no filter at all
        inria_wbc::utils::Timer timer;
        while (simu.scheduler().next_time() < vm["duration"].as<int>() && !simu.graphics()->done()) {

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

            if (vm["height"].as<bool>())
                std::cout << controller->t() << "  floating base height: " << controller->q(false)[2] << " - total feet force: " << ft_sensor_right->force().norm() + ft_sensor_left->force().norm() << std::endl;

            // step the command
            if (simu.schedule(simu.control_freq())) {

                // update the sensors
                // left foot
                if (ft_sensor_left->active()) {
                    sensor_data["lf_torque"] = ft_sensor_left->torque();
                    sensor_data["lf_force"] = ft_sensor_left->force();
                }
                else {
                    sensor_data["lf_torque"] = Eigen::VectorXd::Zero(6);
                    sensor_data["lf_force"] = Eigen::VectorXd::Zero(3);
                }
                // right foot
                if (ft_sensor_right->active()) {
                    sensor_data["rf_torque"] = ft_sensor_right->torque();
                    sensor_data["rf_force"] = ft_sensor_right->force();
                }
                else {
                    sensor_data["rf_torque"] = Eigen::VectorXd::Zero(6);
                    sensor_data["rf_force"] = Eigen::VectorXd::Zero(3);
                }
                // accelerometer
                sensor_data["acceleration"] = imu->linear_acceleration();
                sensor_data["velocity"] = robot->com_velocity().tail<3>();
                // joint positions / velocities (excluding floating base)
                // 0 for joints that are not in dart_joint_names
                Eigen::VectorXd positions = Eigen::VectorXd::Zero(controller->controllable_dofs(false).size());
                Eigen::VectorXd velocities = Eigen::VectorXd::Zero(controller->controllable_dofs(false).size());
                for (size_t i = 0; i < controllable_dofs.size(); ++i) {
                    auto name = controllable_dofs[i];
                    if (std::count(dart_joint_names.begin(), dart_joint_names.end(), name) > 0) {
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

                auto q_no_mimic = controller->filter_cmd(q).tail(ncontrollable);//no fb
                auto q_damaged = filter_cmd(q_no_mimic, controllable_dofs, dart_joint_names);
                timer.begin("cmd");
                if (vm["actuators"].as<std::string>() == "velocity" || vm["actuators"].as<std::string>() == "servo")
                    cmd = inria_wbc::robot_dart::compute_velocities(robot, q_damaged, 1. / control_freq, dart_joint_names);
                else if (vm["actuators"].as<std::string>() == "spd")
                {
                    auto cmd_all = inria_wbc::robot_dart::compute_spd(robot,  filter_cmd(q, controller->all_dofs(false), dart_all_dofs), 1. / sim_freq);
                    //auto cmd_no_mimic = controller->filter_cmd(cmd_all);//rm_from_command(cmd_all, dart_all_dofs, controller->mimic_names());
                    auto cmd_no_mimic = rm_from_command(cmd_all, dart_all_dofs, controller->mimic_names());
                    std::cout<<"cmd no mimic size:"<<cmd_no_mimic.size()<<" dart joints"<<dart_joint_names.size()<<std::endl;
                    cmd = cmd_no_mimic.tail(cmd_no_mimic.size() - 6); // remove the flloating base
                }
                else // torque
                    cmd = controller->tau(false);
                timer.end("cmd");

                if (ghost) {
                    Eigen::VectorXd translate_ghost = Eigen::VectorXd::Zero(6);
                    translate_ghost(0) -= 1;
                    ghost->set_positions(controller->filter_cmd(q).tail(ncontrollable), controllable_dofs);
                    ghost->set_positions(q.head(6) + translate_ghost, floating_base);
                }
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
                auto pforce = vm["norm_force"].as<float>();
                for (auto& p : pv) {
                    if (simu.scheduler().current_time() > p && simu.scheduler().current_time() < p + 0.5) {
                        robot->set_external_force("base_link", Eigen::Vector3d(0, pforce, 0));
                        // robot->set_external_force("base_link", Eigen::Vector3d(pforce, 0, 0));
                        push = true;
                    }
                    if (simu.scheduler().current_time() > p + 0.25)
                        robot->clear_external_forces();
                }
            }

            if (simu.scheduler().current_time() > 2.0 && !damaged) {
                damaged = true;
                std::cout << "DAMAGE!" << std::endl;
                auto body_node = robot->skeleton()->getBodyNode("leg_right_4_link");
                auto tmp_skel = body_node->split("tmp");
                // get the names of the joints that we removed
                std::vector<std::string> removed_joints;
                for (size_t i = 0; i < tmp_skel->getNumJoints(); ++i) {
                    auto name = tmp_skel->getJoint(i)->getName();
                    removed_joints.push_back(name);
                    if (ft_sensor_left->active() && name == ft_sensor_left->attached_to())
                        ft_sensor_left->detach();
                    if (ft_sensor_right->active() && name == ft_sensor_right->attached_to())
                        ft_sensor_right->detach();
                    for (auto& t : torque_sensors)
                        if (t->active() && name == t->attached_to())
                            t->detach();
                }
                robot->update_joint_dof_maps();
                dart_joint_names.clear();
                dart_all_dofs.clear();
                auto j_names = robot->joint_names(); // TODO factorize
                for (auto& x : controllable_dofs)
                    if (std::find(removed_joints.begin(), removed_joints.end(), x) == removed_joints.end())
                        dart_joint_names.push_back(x);
                for (auto& x : controller->all_dofs(false))
                    if (std::find(removed_joints.begin(), removed_joints.end(), x) == removed_joints.end())
                        dart_all_dofs.push_back(x);

            }
            // step the simulation
            {
                timer.begin("sim");
                robot->set_commands(cmd, dart_joint_names);
                simu.step_world();
                timer.end("sim");
            }
            if (traj_saver)
                traj_saver->update();
            // log if needed
            for (auto& x : log_files) {
                if (x.first == "timing")
                    timer.report(*x.second, simu.scheduler().current_time());
                else if (x.first == "cmd")
                    (*x.second) << cmd.transpose() << std::endl;
                else if (x.first == "com") // the real com
                    (*x.second) << robot->com().transpose() << std::endl;
                else if (x.first == "controller_com") // the com according to controller
                    (*x.second) << controller->com().transpose() << std::endl;
                else if (x.first == "cop") // the cop according to controller
                    (*x.second) << controller->cop().transpose() << " "
                                << controller->lcop().transpose() << " "
                                << controller->rcop().transpose() << " " << std::endl;
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
                else if (x.first == "momentum") // the momentum according to pinocchio
                    (*x.second) << controller->momentum().transpose() << std::endl;
                else
                    (*x.second) << robot->body_pose(x.first).translation().transpose() << std::endl;
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
               timer.report(oss, simu.scheduler().current_time(), -1, '\n');
               if (!vm.count("mp4"))
                   simu.set_text_panel(oss.str());
#endif
            }
            timer.report(simu.scheduler().current_time(), 100);
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
