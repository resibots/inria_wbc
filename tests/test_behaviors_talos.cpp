#define BOOST_TEST_MODULE behaviors test
#define BOOST_TEST_MAIN

#include <chrono>
#include <ctime>
#include <iostream>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/test/unit_test.hpp>

#include <robot_dart/robot.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/sensor/force_torque.hpp>
#include <robot_dart/sensor/imu.hpp>
#include <robot_dart/sensor/torque.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

#include "inria_wbc/behaviors/behavior.hpp"
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
    static constexpr double pos_tracking_tsid_tol = 0.03;
    static constexpr double pos_tracking_dart_tol = 0.03;
    static constexpr double com_tracking_tsid_tol = 0.01;
    static constexpr double com_tracking_dart_tol = 0.01;

} // namespace cst

namespace colors {
    static const std::string red = "\x1B[31m";
    static const std::string green = "\x1B[32m";
    static const std::string yellow = "\x1B[33m";
    static const std::string blue = "\x1B[34m";
    static const std::string magenta = "\x1B[35m";
    static const std::string cyan = "\x1B[36m";
    static const std::string rst = "\x1B[0m";
    static const std::string bold = "\x1B[1m";
} // namespace colors

namespace inria_wbc {
    namespace tests {
        struct SE3TaskData {
            std::string name;
            std::string tracked;
            std::string mask;
            double weight;
            double error_dart = 0.0;
            double error_tsid = 0.0;
        };
    } // namespace tests
} // namespace inria_wbc

namespace y = YAML;

// for timestamps
inline std::string date()
{
    char date[30];
    time_t date_time;
    time(&date_time);
    strftime(date, 30, "%Y-%m-%d_%H-%M-%S", localtime(&date_time));
    return std::string(date);
}

// we parse again:
// - to check that the controller parses as expected
// - and to get additional info (mask, etc.)
std::vector<inria_wbc::tests::SE3TaskData> parse_tasks(const std::string& config_path)
{
    std::vector<inria_wbc::tests::SE3TaskData> tasks;
    y::Node config = y::LoadFile(config_path)["CONTROLLER"];
    auto path = boost::filesystem::path(config_path).parent_path();
    auto task_file = config["tasks"].as<std::string>();
    auto p = path / boost::filesystem::path(task_file);
    y::Node task_list = y::LoadFile(p.string());
    for (auto it = task_list.begin(); it != task_list.end(); ++it) {
        auto name = it->first.as<std::string>();
        auto type = it->second["type"].as<std::string>();
        if (type == "se3") {
            auto weight = it->second["weight"].as<double>();
            auto mask = it->second["mask"].as<std::string>();
            auto tracked = it->second["tracked"].as<std::string>();
            tasks.push_back({name, tracked, mask, weight});
        }
    }
    return tasks;
}

void test_behavior(const std::string& config_path,
    robot_dart::RobotDARTSimu& simu,
    const std::shared_ptr<robot_dart::Robot>& robot,
    const std::string& actuator_type,
    y::Emitter& yout)
{
    // ----------------------- init -----------------------
    inria_wbc::controllers::Controller::Params params = {
        robot->model_filename(),
        config_path,
        cst::dt,
        false,
        robot->mimic_dof_names()};

    y::Node config = y::LoadFile(config_path);

    // get the controller
    auto controller_name = config["CONTROLLER"]["name"].as<std::string>();
    auto controller = inria_wbc::controllers::Factory::instance().create(controller_name, params);
    BOOST_CHECK(controller);
    auto p_controller = std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller);
    BOOST_CHECK(p_controller);
    BOOST_CHECK(!p_controller->tasks().empty());

    // get the list of SE3 tasks (to test the tracking)
    auto se3_tasks = parse_tasks(config_path);
    std::cout << "SE3 TASKS (frames): ";
    for (auto& x : se3_tasks)
        std::cout << x.name << "[" << x.tracked << "] ";
    std::cout << std::endl;
    BOOST_CHECK(!se3_tasks.empty());

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

    // ----------------------- the main loop -----------------------
    using namespace std::chrono;
    inria_wbc::controllers::SensorData sensor_data;
    Eigen::VectorXd tq_sensors = Eigen::VectorXd::Zero(torque_sensors.size());

    double error_com_tsid = 0.0;
    double error_com_dart = 0.0;

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
        // compute the CoM error
        error_com_dart += (robot->com() - p_controller->com_task()->getReference().pos).norm();
        error_com_tsid += (p_controller->com() - p_controller->com_task()->getReference().pos).norm();

        // tracking information
        for (auto& t : se3_tasks) {
            Eigen::VectorXd pos_dart, pos_tsid;
            if (robot->joint(t.tracked)) {
                auto node = robot->joint(t.tracked)->getParentBodyNode();
                auto tf_bd_world = node->getWorldTransform();
                auto tf_bd_joint = robot->joint(t.tracked)->getTransformFromParentBodyNode();
                pos_dart = (tf_bd_world * tf_bd_joint).translation();
                pos_tsid = controller->model_joint_pos(t.tracked).translation();
            }
            else {
                pos_dart = robot->body_pose(t.tracked).translation();
                // Q: doesn't TSID create a frame for each joint?
                pos_tsid = controller->model_frame_pos(t.tracked).translation();
            }
            // masks? // TODO also compute angular error
            auto pos_ref = p_controller->get_se3_ref(t.name).translation();
            if (t.weight > 0) {
                double error_dart = 0.;
                double error_tsid = 0.;
                BOOST_CHECK(t.mask.size() == 6);
                for (int i = 0; i < 3; ++i) {
                    if (t.mask[i] == '1') {
                        error_dart += (pos_dart[i] - pos_ref[i]) * (pos_dart[i] - pos_ref[i]);
                        error_tsid += (pos_tsid[i] - pos_ref[i]) * (pos_tsid[i] - pos_ref[i]);
                    }
                }
                t.error_dart += sqrt(error_dart);
                t.error_tsid += sqrt(error_tsid);
            }
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
              << "\ttotal:" << (time_simu + time_cmd + time_solver) / 1e6 << " s"
              << "\t[" << (t_sim + t_cmd + t_solver) << " ms / it.]"
              << "\tsimu: " << t_sim << " ms/it"
              << "\tsolver:" << t_solver << " ms/it"
              << "\tcmd:" << t_cmd << " ms/it"
              << std::endl;

    yout << y::Key << "time" << y::Value
         << std::vector<double>{(time_simu + time_cmd + time_solver) / 1e6, (t_sim + t_cmd + t_solver), t_sim, t_solver, t_cmd};

    // error report for COM
    error_com_tsid /= simu.scheduler().current_time() * 1000;
    error_com_dart /= simu.scheduler().current_time() * 1000;
    std::vector<double> com_errors = {error_com_tsid, error_com_dart};
    std::cout << "CoM: [tsid]" << error_com_tsid << " [dart]" << error_com_dart << std::endl;
    BOOST_CHECK(error_com_tsid < cst::com_tracking_tsid_tol);
    BOOST_CHECK(error_com_dart < cst::com_tracking_dart_tol);
    yout << y::Key << "com" << y::Value << std::vector<double>{error_com_tsid, error_com_dart};

    // error report for se3 tasks
    for (auto& t : se3_tasks) {
        t.error_tsid /= simu.scheduler().current_time() * 1000;
        t.error_dart /= simu.scheduler().current_time() * 1000;
        std::cout << "task: " << t.name << " [" << t.tracked << "]"
                  << "pos error TSID:" << t.error_tsid << " "
                  << "pos error DART:" << t.error_dart << " "
                  << "\t(mask =" << t.mask << ",weight=" << t.weight << ")" << std::endl;
        yout << y::Key << t.name << y::Value << std::vector<double>{t.error_tsid, t.error_dart};
        BOOST_CHECK(t.error_tsid < cst::pos_tracking_tsid_tol);
        BOOST_CHECK(t.error_dart < cst::pos_tracking_dart_tol);
    }

    std::cout << std::endl;
}

void test_behavior(const std::string& config_path, const std::string& actuators, const std::string& coll, bool fast, y::Emitter& yout)
{
    std::string urdf = fast ? "talos/talos_fast.urdf" : "talos/talos.urdf";
    std::cout << colors::blue << colors::bold << "TESTING: " << config_path << " | " << actuators << " | " << coll << " | " << urdf << colors::rst << std::endl;
    yout << y::Key << urdf << y::BeginMap;

    // create the simulator and the robot
    std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
    auto robot = std::make_shared<robot_dart::Robot>(urdf, packages);
    robot->set_actuator_types(actuators);
    robot_dart::RobotDARTSimu simu(cst::dt);
    simu.set_collision_detector(coll);
    simu.set_control_freq(cst::frequency);
    simu.add_robot(robot);
    simu.add_checkerboard_floor();

#ifdef GRAPHIC
    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>();
    simu.set_graphics(graphics);
    graphics->look_at({3.5, -2, 2.2}, {0., 0., 1.4});
    // we always save the video (useful for bug reports)
    graphics->record_video("test_result_talos.mp4");
#endif

    try {
        // run the behavior
        test_behavior(config_path, simu, robot, actuators, yout);
    }
    catch (std::exception& e) {
        std::cout << colors::red << colors::bold << "Error (exception): t=" << simu.scheduler().current_time() << " " << colors::rst << e.what() << std::endl;
        BOOST_CHECK(e.what());
    }
    yout << y::EndMap;
}

BOOST_AUTO_TEST_CASE(behaviors)
{
    // we write the results in yaml file, for future comparisons and analysis
    y::Emitter yout;
    yout << y::BeginMap;

    // the YAMl file has a timestamp (so that we can archive them)
    yout << y::Key << "timestamp" << y::Value << date();

    // use ./my_test -- ../../etc/arm.yaml servo fcl 0 for a specific test
    auto argc = boost::unit_test::framework::master_test_suite().argc;
    auto argv = boost::unit_test::framework::master_test_suite().argv;

    if (argc > 1) {
        assert(argc == 5);
        std::cout << "using custom arguments for test" << std::endl;
        test_behavior(argv[1], argv[2], argv[3], std::stoi(argv[4]), yout);
        // WARING: this does not write the yaml file (it would be in)
    }

    ///// the default behavior is to run all the combinations and write the output in the yaml file

    // this is relative to the "tests" directory
    auto behaviors = {"../../etc/arm.yaml", "../../etc/squat.yaml", "../../etc/talos_clapping.yaml", "../../etc/walk_on_spot.yaml"};
    auto collision = {"fcl", "dart"};
    auto actuators = {"servo", "torque", "velocity"};

    for (auto& b : behaviors) {
        yout << y::Key << b << y::Value << y::BeginMap;
        for (auto& c : collision) {
            yout << y::Key << c << y::Value << y::BeginMap;
            for (auto& a : actuators) {
                yout << y::Key << a << y::Value << y::BeginMap;
                test_behavior(b, a, c, true, yout);
                if (c != std::string("dart")) {
                    test_behavior(b, a, c, false, yout);
                }
                yout << y::EndMap;
                std::cout << yout.c_str() << std::endl;
            }
            yout << y::EndMap;
        }
        yout << y::EndMap;
    }
    yout << y::EndMap;
    auto fname = std::string("test_results-") + date() + ".yaml";
    std::ofstream ofs(fname.c_str());
    ofs << yout.c_str();
}