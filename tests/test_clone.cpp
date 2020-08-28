#define BOOST_TEST_MODULE clone test
#include <boost/test/unit_test.hpp>

#include <inria_wbc/behaviors/factory.hpp>
#include <iostream>
#include <robot_dart/robot.hpp>
#include <vector>

std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> test_behavior(
    const std::shared_ptr<inria_wbc::behaviors::Behavior>& behavior)
{
    std::cout << "running...";
    std::cout.flush();
    auto controller = behavior->controller();

    auto all_dofs = controller->all_dofs();
    auto controllable_dofs = controller->controllable_dofs();
    uint ncontrollable = controllable_dofs.size();

    std::vector<Eigen::VectorXd> cmds, cmds_filtered;
    Eigen::VectorXd cmd;
    bool is_valid = false;
    for (int i = 0; i < 3; ++i) {
        is_valid = behavior->cmd(cmd);
        if(is_valid)
        {
            cmds.push_back(cmd);
            cmds_filtered.push_back(controller->filter_cmd(cmd).tail(ncontrollable));
        }
    }
    std::cout << "done" << std::endl;
    return std::make_pair(cmds, cmds_filtered);
}

template <typename T> void compare_cmds(const T& cmds, const T& cmds2)
{
    BOOST_CHECK_EQUAL(cmds.first.size(), cmds2.first.size());
    for (int i = 0; i < cmds.first.size(); ++i) {
        BOOST_CHECK(cmds.first[i].isApprox(cmds2.first[i], 1e-8));
        BOOST_CHECK(cmds.second[i].isApprox(cmds2.second[i], 1e-8));
    }
}

BOOST_AUTO_TEST_CASE(clone_test)
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
        auto behavior2 = behavior->clone();

        auto cmds = test_behavior(behavior);
        auto cmds2 = test_behavior(behavior2);
    
        compare_cmds(cmds, cmds2);
    }
}
