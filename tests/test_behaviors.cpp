#define BOOST_TEST_MODULE clone test
#include <boost/test/unit_test.hpp>

#include <inria_wbc/behaviors/behavior.hpp>
#include <iostream>
#include <robot_dart/robot.hpp>
#include <vector>

#include "test_behavior.hpp"

BOOST_AUTO_TEST_CASE(running)
{
    // we simply check that things are running without error / segv
    srand(time(0));
    auto behaviors = {"../etc/squat.yaml", "../etc/arm.yaml"};
    std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
    auto robot = std::make_shared<robot_dart::Robot>("talos/talos.urdf", packages);

    std::cout << "robot:" << robot->model_filename() << std::endl;

    for (auto& sot_config_path : behaviors) {
        std::cout << "configuration:" << sot_config_path << std::endl;

        inria_wbc::controllers::Controller::Params params = {
            robot->model_filename(),
            sot_config_path,
            true,
            0.001,
            false,
            robot->mimic_dof_names()};

        std::string behavior_name, controller_name;
        YAML::Node config = YAML::LoadFile(sot_config_path);
        inria_wbc::utils::parse(behavior_name, "name", config, "BEHAVIOR", true);
        inria_wbc::utils::parse(controller_name, "name", config, "CONTROLLER", true);

        auto controller = inria_wbc::controllers::Factory::instance().create(controller_name, params);
        auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, controller);

        auto cmds = test_behavior(behavior);
    }
}
