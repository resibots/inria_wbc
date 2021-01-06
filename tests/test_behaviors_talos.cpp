#define BOOST_TEST_MODULE behaviors test

#include <boost/test/unit_test.hpp>
#include <chrono>

#include <inria_wbc/behaviors/behavior.hpp>
#include <iostream>
#include <robot_dart/robot.hpp>

#include <vector>

#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/sensor/force_torque.hpp>
#include <robot_dart/sensor/imu.hpp>
#include <robot_dart/sensor/torque.hpp>

#include "inria_wbc/robot_dart/cmd.hpp"

#include "inria_wbc/behaviors/behavior.hpp"
#include "inria_wbc/controllers/pos_tracker.hpp"
#include "inria_wbc/controllers/talos_pos_tracker.hpp"
#include "inria_wbc/exceptions.hpp"
#include "inria_wbc/robot_dart/cmd.hpp"
#include "inria_wbc/robot_dart/self_collision_detector.hpp"

namespace cst {
    static constexpr double dt = 0.001;
    static constexpr double duration = 10;
    static constexpr double frequency = 1000;

} // namespace cst

void test_behavior(const std::string& config_path,
    robot_dart::RobotDARTSimu& simu,
    const std::shared_ptr<robot_dart::Robot>& robot,
    const std::string& actuator_type)
{
    inria_wbc::controllers::Controller::Params params = {
        robot->model_filename(),
        config_path,
        cst::dt,
        false,
        robot->mimic_dof_names()};

    YAML::Node config = YAML::LoadFile(config_path);

    // get the controller
    auto controller_name = config["CONTROLLER"]["name"].as<std::string>();
    auto controller = inria_wbc::controllers::Factory::instance().create(controller_name, params);
    BOOST_CHECK(controller);

    // get the behavior (trajectories)
    auto behavior_name = config["BEHAVIOR"]["name"].as<std::string>();
    auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, controller);
    BOOST_CHECK(behavior);

    // add sensors to the robot (robot_dart)
    // Force/torque (feet)
    auto ft_sensor_left = simu.add_sensor<robot_dart::sensor::ForceTorque>(robot, "leg_left_6_joint");
    auto ft_sensor_right = simu.add_sensor<robot_dart::sensor::ForceTorque>(robot, "leg_right_6_joint");
    // IMU
    robot_dart::sensor::IMUConfig imu_config;
    imu_config.body = robot->body_node("imu_link");
    imu_config.frequency = cst::frequency;
    auto imu = simu.add_sensor<robot_dart::sensor::IMU>(imu_config);
    // Torque (joints)
    std::vector<std::shared_ptr<robot_dart::sensor::Torque>> torque_sensors;
    auto talos_tracker_controller = std::static_pointer_cast<inria_wbc::controllers::TalosPosTracker>(controller);
    for (const auto& joint : talos_tracker_controller->torque_sensor_joints())
        torque_sensors.push_back(simu.add_sensor<robot_dart::sensor::Torque>(robot, joint, cst::frequency));

    // a few useful variables
    auto all_dofs = controller->all_dofs();
    auto floating_base = all_dofs;
    floating_base.resize(6);
    auto controllable_dofs = controller->controllable_dofs();
    uint ncontrollable = controllable_dofs.size();
    double time_simu = 0, time_cmd = 0, time_solver = 0;
    int it_simu = 0, it_cmd = 0;

    // init the robot
    robot->set_positions(controller->q0(), all_dofs);

    // the main loop
    using namespace std::chrono;
    inria_wbc::controllers::SensorData sensor_data;
    Eigen::VectorXd tq_sensors = Eigen::VectorXd::Zero(torque_sensors.size());

    Eigen::VectorXd cmd;
    while (simu.scheduler().next_time() < cst::duration) {
        double time_step_solver = 0, time_step_cmd = 0, time_step_simu = 0;

        // update the sensors from the simulator
        for (auto tq_sens = torque_sensors.cbegin(); tq_sens < torque_sensors.cend(); ++tq_sens)
            tq_sensors(std::distance(torque_sensors.cbegin(), tq_sens)) = (*tq_sens)->torques()(0, 0);
        sensor_data["joints_torque"] = tq_sensors;
        sensor_data["lf_torque"] = ft_sensor_left->torque();
        sensor_data["lf_force"] = ft_sensor_left->force();
        sensor_data["rf_torque"] = ft_sensor_right->torque();
        sensor_data["rf_force"] = ft_sensor_right->force();
        sensor_data["acceleration"] = imu->linear_acceleration();
        sensor_data["velocity"] = robot->com_velocity().tail<3>();
        sensor_data["positions"] = robot->skeleton()->getPositions().tail(ncontrollable);
        ;

        // command
        if (simu.schedule(simu.control_freq())) {
            auto t1_solver = high_resolution_clock::now();
            behavior->update(sensor_data);
            auto q = controller->q(false);
            auto t2_solver = high_resolution_clock::now();
            time_step_solver = duration_cast<microseconds>(t2_solver - t1_solver).count();

            auto t1_cmd = high_resolution_clock::now();
            if (actuator_type == "velocity" || actuator_type == "servo")
                cmd = inria_wbc::robot_dart::compute_velocities(robot->skeleton(), q, cst::dt);
            else // torque
                cmd = inria_wbc::robot_dart::compute_spd(robot->skeleton(), q);
            auto t2_cmd = high_resolution_clock::now();
            time_step_cmd = duration_cast<microseconds>(t2_cmd - t1_cmd).count();

            robot->set_commands(controller->filter_cmd(cmd).tail(ncontrollable), controllable_dofs);
            ++it_cmd;
        }

        // step the simulation
        {
            auto t1_simu = high_resolution_clock::now();
            simu.step_world();
            auto t2_simu = high_resolution_clock::now();
            time_step_simu = duration_cast<microseconds>(t2_simu - t1_simu).count();
            ++it_simu;
        }
        // timing information
        time_simu += time_step_simu;
        time_cmd += time_step_cmd;
        time_solver += time_step_solver;
    }
    // time report
    double t_sim = time_simu / it_simu / 1000.;
    double t_cmd = time_cmd / it_cmd / 1000.;
    double t_solver = time_solver / it_cmd / 1000.;

    std::cout << "TIME: "
              << "\t" << (time_simu + time_cmd + time_solver) / 1e6 << " s"
              << "\tit. simu: " << t_sim << " ms"
              << "\tit. solver:" << t_solver << " ms"
              << "\tit. cmd:" << t_cmd << " ms"
              << std::endl;
}

void test_behavior(const std::string& config_path, const std::string& actuators, const std::string& coll, bool fast)
{
    std::string urdf = fast ? "talos/talos_fast.urdf" : "talos/talos.urdf";
    std::cout << "TESTING: " << config_path << " | " << actuators << " | " << coll << " | " << urdf << std::endl;

    // create the simulator and the robot
    std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
    auto robot = std::make_shared<robot_dart::Robot>(urdf, packages);
    robot->set_actuator_types(actuators);
    robot_dart::RobotDARTSimu simu(cst::dt);
    simu.set_collision_detector(coll);
    simu.set_control_freq(cst::frequency);
    simu.add_robot(robot);
    simu.add_checkerboard_floor();
    
    // run the behavior
    test_behavior(config_path, simu, robot, actuators);
}

BOOST_AUTO_TEST_CASE(behaviors)
{
    // this is relative to the "tests" directory
    auto behaviors = {"../../etc/squat.yaml", "../../etc/arm.yaml", "../../etc/talos_clapping.yaml"};
    auto collision = {"fcl", "dart"};
    auto actuators = {"servo", "torque", "velocity"};

    for (auto& b : behaviors)
        for (auto& c : collision)
            for (auto& a : actuators) {
                test_behavior(b, a, c, true);
                if (c != std::string("dart"))
                    test_behavior(b, a, c, false);
            }
}