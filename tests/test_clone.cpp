
#include <inria_wbc/behaviors/factory.hpp>
#include <iostream>
#include <robot_dart/robot.hpp>
#include <vector>

std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> test_behavior(
    const std::shared_ptr<inria_wbc::behaviors::Behavior>& behavior)
{
    std::cout<<"running..."; std::cout.flush();
    auto controller = behavior->controller();
    auto all_dofs = controller->all_dofs();
    auto controllable_dofs = controller->controllable_dofs();
    uint ncontrollable = controllable_dofs.size();

    std::vector<Eigen::VectorXd> cmds, cmds_filtered;
    for (int i = 0; i < 10; ++i) {
        std::cout<<"before"<<std::endl;
        auto cmd = behavior->cmd();
        std::cout<<"cmd:"<<cmd<<std::endl;
        cmds.push_back(cmd);
        cmds_filtered.push_back(controller->filter_cmd(cmd).tail(ncontrollable));
    }
    std::cout<<"done"<<std::endl;
    return std::make_pair(cmds, cmds_filtered);
}

int main(int argc, char* argv[])
{
    // take the name of the behavior as a argument
    std::string sot_config_path = argc > 1 ? argv[1] : "../etc/squat.yaml";

    std::vector<std::pair<std::string, std::string>> packages
        = {{"talos_description", "talos/talos_description"}};
    auto robot = std::make_shared<robot_dart::Robot>("talos/talos.urdf", packages);

    inria_wbc::controllers::TalosBaseController::Params params
        = {robot->model_filename(), "../etc/talos_configurations.srdf", sot_config_path, "", 0.001,
            false, robot->mimic_dof_names()};

    std::cout << "configuration:" << sot_config_path << std::endl;
    std::cout << "robot:" << robot->model_filename() << std::endl;

    std::string behavior_name;
    auto config = YAML::LoadFile(sot_config_path);
    inria_wbc::utils::parse(behavior_name, "name", config, false, "BEHAVIOR");

    auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, params);
    auto cmds = test_behavior(behavior);

    return 0;
}
