#define BOOST_TEST_MODULE clone test
#include <boost/test/unit_test.hpp>

#include <inria_wbc/behaviors/factory.hpp>
#include <iostream>
#include <robot_dart/robot.hpp>
#include <vector>

#include "test_behavior.hpp"

BOOST_AUTO_TEST_CASE(factory_test)
{
    srand(time(0));
    auto behaviors = {"../etc/squat.yaml", "../etc/arm.yaml"};
    std::vector<std::pair<std::string, std::string>> packages
        = {{"talos_description", "talos/talos_description"}};
    auto robot = std::make_shared<robot_dart::Robot>("talos/talos.urdf", packages);

    std::cout << "robot:" << robot->model_filename() << std::endl;

    for (auto& sot_config_path : behaviors) {
        std::cout << "configuration:" << sot_config_path << std::endl;

        inria_wbc::controllers::TalosBaseController::Params params
            = {robot->model_filename(), "../etc/talos_configurations.srdf", sot_config_path, "",
                0.001, false, robot->mimic_dof_names()};

        std::string behavior_name;
        auto config = YAML::LoadFile(sot_config_path);
        inria_wbc::utils::parse(behavior_name, "name", config, false, "BEHAVIOR");

        auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, params);
        auto behavior2 = inria_wbc::behaviors::Factory::instance().create(behavior_name, params);
        auto behavior3 = inria_wbc::behaviors::Factory::instance().create(behavior_name, params);


        auto cmds = test_behavior(behavior);
        auto cmds2 = test_behavior(behavior2);
        auto cmds3 = test_behavior(behavior3);

        compare_cmds(cmds, cmds2);
        compare_cmds(cmds, cmds2);
    }
}
