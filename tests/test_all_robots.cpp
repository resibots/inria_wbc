#include <chrono>
#include <ctime>
#include <iostream>
#include <map>
#include <stdio.h>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/robots/franka.hpp>
#include <robot_dart/robots/talos.hpp>
#include <robot_dart/robots/icub.hpp>

#include "inria_wbc/behaviors/behavior.hpp"
#include "inria_wbc/robot_dart/cmd.hpp"

#include "inria_wbc/behaviors/behavior.hpp"
#include "inria_wbc/controllers/pos_tracker.hpp"
#include "inria_wbc/controllers/talos_pos_tracker.hpp"

#include "inria_wbc/exceptions.hpp"
#include "inria_wbc/robot_dart/cmd.hpp"
#include "inria_wbc/robot_dart/external_collision_detector.hpp"
#include "inria_wbc/robot_dart/self_collision_detector.hpp"
#include "inria_wbc/robot_dart/utils.hpp"
#include "inria_wbc/utils/timer.hpp"

#include "utest.hpp"

// parameters of the test suite
// could be tuned as arguments of the test suite
namespace cst {
    static double sim_dt = 0.001;
    static double duration = 2; // length of each test
    static double ctrl_frequency = 1000; // 1000 Hz
} // namespace cst

namespace y = YAML;

// this is the only part that is robot specific
inria_wbc::controllers::SensorData sensor_data_franka(const std::shared_ptr<robot_dart::Robot>& robot, 
std::shared_ptr<inria_wbc::controllers::Controller>& controller)
{
    robot->fix_to_world(); // this is robot-specific too

    inria_wbc::controllers::SensorData sensor_data;
    // left foot
    sensor_data["lf_torque"] = Eigen::Vector3d::Zero();
    sensor_data["lf_force"] = Eigen::Vector3d::Zero();
    // right foot
    sensor_data["rf_torque"] = Eigen::Vector3d::Zero();
    sensor_data["rf_force"] = Eigen::Vector3d::Zero();
    // accelerometer
    sensor_data["acceleration"] = Eigen::Vector3d::Zero();
    sensor_data["velocity"] = Eigen::Vector3d::Zero();
    // joint positions (excluding floating base)
    int ncontrollable = controller->controllable_dofs().size();
    sensor_data["positions"] = robot->skeleton()->getPositions().tail(ncontrollable);
    return sensor_data;
}

// this is the only part that is robot specific
inria_wbc::controllers::SensorData sensor_data_talos(const std::shared_ptr<robot_dart::Robot>& r, std::shared_ptr<inria_wbc::controllers::Controller>& c)
{
    auto controller = std::dynamic_pointer_cast<inria_wbc::controllers::TalosPosTracker>(c);
    assert(controller);
    auto robot = std::dynamic_pointer_cast<robot_dart::robots::Talos>(r);
    assert(robot);

    inria_wbc::controllers::SensorData sensor_data;

    // update the sensors from the simulator
    Eigen::VectorXd torques = Eigen::VectorXd::Zero(controller->torque_sensor_joints().size());
    for (size_t i = 0; i < controller->torque_sensor_joints().size(); ++i)
        torques[i] = robot->torques().at(controller->torque_sensor_joints()[i])->torques()(0, 0);
    sensor_data["joints_torque"] = torques;
    sensor_data["lf_torque"] = robot->ft_foot_left().torque();
    sensor_data["lf_force"] = robot->ft_foot_left().force();
    sensor_data["rf_torque"] = robot->ft_foot_right().torque();
    sensor_data["rf_force"] = robot->ft_foot_right().force();
    sensor_data["velocity"] = robot->com_velocity().tail<3>();
    sensor_data["positions"] = robot->positions(controller->controllable_dofs(false));
    sensor_data["joint_velocities"] = robot->velocities(controller->controllable_dofs(false));
    sensor_data["floating_base_position"] = inria_wbc::robot_dart::floating_base_pos(robot->positions());
    sensor_data["floating_base_velocity"] = inria_wbc::robot_dart::floating_base_vel(robot->velocities());
    sensor_data["imu_pos"] = robot->imu().angular_position_vec();
    sensor_data["imu_vel"] = robot->imu().angular_velocity();
    sensor_data["imu_acc"] = robot->imu().linear_acceleration();

    return sensor_data;
}


// this is the only part that is robot specific
inria_wbc::controllers::SensorData sensor_data_icub(const std::shared_ptr<robot_dart::Robot>& r, std::shared_ptr<inria_wbc::controllers::Controller>& c)
{
    auto controller = std::dynamic_pointer_cast<inria_wbc::controllers::HumanoidPosTracker>(c);
    assert(controller);
    auto robot = std::dynamic_pointer_cast<robot_dart::robots::ICub>(r);
    assert(robot);

    inria_wbc::controllers::SensorData sensor_data;

    sensor_data["lf_torque"] = robot->ft_foot_left().torque();
    sensor_data["lf_force"] = robot->ft_foot_left().force();
    sensor_data["rf_torque"] = robot->ft_foot_right().torque();
    sensor_data["rf_force"] = robot->ft_foot_right().force();
    sensor_data["velocity"] = robot->com_velocity().tail<3>();
    sensor_data["positions"] = robot->positions(controller->controllable_dofs(false));
    sensor_data["joint_velocities"] = robot->velocities(controller->controllable_dofs(false));
    sensor_data["floating_base_position"] = inria_wbc::robot_dart::floating_base_pos(robot->positions());
    sensor_data["floating_base_velocity"] = inria_wbc::robot_dart::floating_base_vel(robot->velocities());
    sensor_data["imu_pos"] = robot->imu().angular_position_vec();
    sensor_data["imu_vel"] = robot->imu().angular_velocity();
    sensor_data["imu_acc"] = robot->imu().linear_acceleration();

    return sensor_data;
}

template <typename FData>
void test_behavior(utest::test_t test,
    const std::string& base_path,
    const std::string& controller_path,
    const std::string& behavior_path,
    const std::shared_ptr<robot_dart::Robot>& robot,
    const FData& fdata,
    bool verbose = false)
{
    try {
        inria_wbc::utils::Timer timer;

        // simulator
        robot->set_actuator_types("servo"); // this one should never fail
        robot->set_position_enforced(true);

        robot_dart::RobotDARTSimu simu(cst::sim_dt);
        simu.set_collision_detector("fcl"); // this is the most generic one
        simu.set_control_freq(cst::ctrl_frequency);
        simu.add_robot(robot);
        simu.add_checkerboard_floor();

        // ----------------------- init -----------------------
        y::Node c_config = IWBC_CHECK(y::LoadFile(base_path + controller_path));
        c_config["CONTROLLER"]["base_path"] = base_path;
        c_config["CONTROLLER"]["urdf"] = robot->model_filename();
        c_config["CONTROLLER"]["mimic_dof_names"] = robot->mimic_dof_names();
        c_config["CONTROLLER"]["verbose"] = verbose;

        // get the controller
        auto controller_name = IWBC_CHECK(c_config["CONTROLLER"]["name"].as<std::string>());
        auto controller = inria_wbc::controllers::Factory::instance().create(controller_name, c_config);
        UTEST_CHECK(test, controller.get() != 0);
        auto p_controller = std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller);
        UTEST_CHECK(test, p_controller.get() != 0);
        UTEST_CHECK(test, !p_controller->tasks().empty());

        // get the behavior (trajectories)
        y::Node b_config = IWBC_CHECK(y::LoadFile(base_path + behavior_path));
        auto behavior_name = IWBC_CHECK(b_config["BEHAVIOR"]["name"].as<std::string>());
        auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, controller, b_config);
        UTEST_CHECK(test, behavior.get() != 0);

        // a few useful variables
        auto controllable_dofs = controller->controllable_dofs();
        uint ncontrollable = controllable_dofs.size();

        // init the robot
        UTEST_CHECK(test, controller->q0().size() == controller->all_dofs().size());
        robot->set_positions(controller->q0(), controller->all_dofs());

        // ----------------------- the main loop -----------------------
        using namespace std::chrono;
        inria_wbc::controllers::SensorData sensor_data;

        Eigen::VectorXd cmd;

        while (simu.scheduler().next_time() < cst::duration) {
            // update the sensors
            sensor_data = fdata(robot, controller);

            // command
            if (simu.schedule(simu.control_freq())) {
                timer.begin("controller");
                behavior->update(sensor_data);
                timer.end("controller");
                auto q = controller->q(false);
                cmd = inria_wbc::robot_dart::compute_velocities(robot, q, cst::sim_dt, controller->all_dofs(false));
                if (simu.scheduler().current_time() == 0)
                    UTEST_CHECK(test, controller->filter_cmd(cmd).tail(ncontrollable).size() == controllable_dofs.size());
                robot->set_commands(controller->filter_cmd(cmd).tail(ncontrollable), controllable_dofs);
            }

            // step the simulation
            {
                simu.step_world();
            }
        }
        auto t = timer["controller"];
        UTEST_INFO(test, "solver + behavior:" + std::to_string(t.time / t.iterations / 1000) + "ms " + "[" + std::to_string(t.min_time / 1000) + "," + std::to_string(t.max_time / 1000) + "]");
    }
    catch (std::exception& e) {
        UTEST_ERROR(test, std::string("error in ref comparison:") + e.what());
    }
}

int main(int argc, char** argv)
{

    ////////////////// program options
    namespace po = boost::program_options;
    po::options_description desc("Test_robot options");
    // clang-format off
    desc.add_options()
        ("n_threads,n", po::value<int>()->default_value(-1), "run tests in parallel (default = number of cores)")
        ("single,s", po::value<std::vector<std::string> >()->multitoken(), "run a single test, args: robot_name controller_path behavior_path actuators")
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

    int n_threads = vm["n_threads"].as<int>();

    utest::TestSuite test_suite;
    // paths are relative to the "tests" directory (to use make check)

    std::string path = "../../etc/";

    /////////// Franka robot
    {
        std::string base_path = path + "/franka/";
        std::string name = "franka";
        std::string controller_path = "pos_tracker.yaml";
        auto behaviors = {"cartesian_line.yaml"};
        for (auto& behavior_path : behaviors) {
            auto test1 = utest::make_test("[" + name + "] " + behavior_path);
            auto franka = std::make_shared<robot_dart::robots::Franka>();
            UTEST_REGISTER(test_suite, test1, test_behavior(test1, base_path, controller_path, behavior_path, franka, sensor_data_franka, false));
        }
    }
    /////////// Talos robot
    {
        std::string base_path = path + "/talos/";
        std::string controller_path = "talos_pos_tracker.yaml";
        auto behaviors = { "arm.yaml",
            "clapping.yaml",
            "squat.yaml",
            "walk.yaml",
            "walk_on_spot.yaml",
            "traj_teleop1.yaml"};
        std::string name = "talos";
        for (auto& behavior_path : behaviors) {
            auto test1 = utest::make_test("[" + name + "] " + behavior_path);
            auto talos = std::make_shared<robot_dart::robots::Talos>();
            UTEST_REGISTER(test_suite, test1, test_behavior(test1, base_path, controller_path, behavior_path, talos, sensor_data_talos, false));
        }
    }

    /////////// Icub robot
    {
        std::string base_path = path + "/icub/";
        std::string controller_path = "humanoid_pos_tracker.yaml";
        auto behaviors = {"arm.yaml",
            "squat.yaml",
            "walk.yaml",
            "walk_on_spot.yaml",
            "traj_teleop1.yaml"};
        std::string name = "icub";
        for (auto& behavior_path : behaviors) {
            auto test1 = utest::make_test("[" + name + "] " + behavior_path);
            auto icub = std::make_shared<robot_dart::robots::ICub>();
            UTEST_REGISTER(test_suite, test1, test_behavior(test1, base_path, controller_path, behavior_path, icub, sensor_data_icub, false));
        }
    }
    ///////// RUN everything
    test_suite.run(n_threads);
    utest::write_report(test_suite, std::cout, true);
    std::cout << "------------ SUMMARY ------------" << std::endl;
    utest::write_report(test_suite, std::cout, false);

    if (test_suite.success())
        return 0;
    else
        return 1;
    return 0;
}
