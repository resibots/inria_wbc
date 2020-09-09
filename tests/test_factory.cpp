#define BOOST_TEST_MODULE clone test
#include <boost/test/unit_test.hpp>

#include <inria_wbc/behaviors/factory.hpp>
#include <iostream>
#include <robot_dart/robot.hpp>
#include <vector>
#include <chrono>

#include "test_behavior.hpp"

BOOST_AUTO_TEST_CASE(factory_test)
{
    srand(time(0));
    auto behaviors = {"../etc/squat.yaml", "../etc/arm.yaml"};
    std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
    auto robot = std::make_shared<robot_dart::Robot>("talos/talos.urdf", packages);

    std::cout << "robot:" << robot->model_filename() << std::endl;

    for (auto &sot_config_path : behaviors)
    {
        std::cout << "configuration:" << sot_config_path << std::endl;

        inria_wbc::controllers::TalosBaseController::Params params = {robot->model_filename(), "../etc/talos_configurations.srdf", sot_config_path, "",
                                                                      0.001, false, robot->mimic_dof_names()};

        std::string behavior_name;
        auto config = YAML::LoadFile(sot_config_path);
        inria_wbc::utils::parse(behavior_name, "name", config, false, "BEHAVIOR");

        // for reference
        auto t1_base = std::chrono::high_resolution_clock::now();
        auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, params);
        auto t2_base = std::chrono::high_resolution_clock::now();
        std::cout << "Standard creation:" << std::chrono::duration_cast<std::chrono::microseconds>(t2_base - t1_base).count() / 1000.0 << " ms" << std::endl;

        // with clone system
        auto t1 = std::chrono::high_resolution_clock::now();
        auto behavior1 = inria_wbc::behaviors::Factory::instance().create_or_clone(behavior_name, params);
        auto t2 = std::chrono::high_resolution_clock::now();
        std::cout << "First clone creation:" << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0 << " ms" << std::endl;

        auto t3 = std::chrono::high_resolution_clock::now();
        auto behavior2 = inria_wbc::behaviors::Factory::instance().create_or_clone(behavior_name, params);
        auto t4 = std::chrono::high_resolution_clock::now();
        std::cout << "Clone creation:" << std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() / 1000.0 << " ms" << std::endl;

        auto cmds_base = test_behavior(behavior);
        auto cmds1 = test_behavior(behavior1);        
        auto cmds2 = test_behavior(behavior2);

        auto behavior3 = inria_wbc::behaviors::Factory::instance().create_or_clone(behavior_name, params);
        auto cmds3 = test_behavior(behavior3);

        compare_cmds(cmds_base, cmds1);
        compare_cmds(cmds_base, cmds2);
        compare_cmds(cmds_base, cmds3);
        
    }
}
