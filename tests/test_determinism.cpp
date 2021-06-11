#define BOOST_TEST_MODULE determinism
#define BOOST_TEST_MAIN

#include <map>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/test/unit_test.hpp>

#include <robot_dart/robot.hpp>

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
namespace y = YAML;

std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> inline test_behavior(
    const std::shared_ptr<inria_wbc::behaviors::Behavior>& behavior, int duration)
{
    std::cout << "running...";
    std::cout.flush();
    auto controller = behavior->controller();

    auto all_dofs = controller->all_dofs();
    auto controllable_dofs = controller->controllable_dofs();
    uint ncontrollable = controllable_dofs.size();

    std::vector<Eigen::VectorXd> cmds, cmds_filtered;
    inria_wbc::controllers::SensorData sensors;
    for (int i = 0; i < duration; ++i) {
        // std::cout << i << " ";
        std::cout.flush();
        behavior->update(sensors); // default values for sensors
        auto cmd = controller->q(false);
        cmds.push_back(cmd);
        cmds_filtered.push_back(controller->filter_cmd(cmd).tail(ncontrollable));
    }
    std::cout << "done" << std::endl;
    return std::make_pair(cmds, cmds_filtered);
}

template <typename T>
inline void compare_cmds(const T& cmds, const T& cmds2)
{
    std::cout << "Comparing commands:";
    BOOST_CHECK_EQUAL(cmds.first.size(), cmds2.first.size());
    for (int i = 0; i < cmds.first.size(); ++i) {
        // std::cout << i << " ";
        std::cout.flush();
        BOOST_CHECK(cmds.first[i].isApprox(cmds2.first[i], 1e-8));
        BOOST_CHECK(cmds.second[i].isApprox(cmds2.second[i], 1e-8));
    }
    std::cout << "done" << std::endl;
}

BOOST_AUTO_TEST_CASE(determinism)
{

    std::string controller_path = "../../etc/talos/talos_pos_tracker.yaml";
    std::vector<std::string> behaviors_paths = {"../../etc/talos/arm.yaml", "../../etc/talos/squat.yaml", "../../etc/talos/clapping.yaml", "../../etc/talos/walk_on_spot.yaml"};
    std::vector<std::string> urdfs = {"talos/talos.urdf", "talos/talos_fast.urdf"};

    std::vector<std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>>> results;
    int duration = 5000;
    int nexp = 5;

    for (int i = 0; i < nexp; i++) {
        y::Node c_config = IWBC_CHECK(y::LoadFile(controller_path));
        c_config["CONTROLLER"]["stabilizer"]["activated"] = false;
        c_config["CONTROLLER"]["collision_detection"]["activated"] = false;
        c_config["CONTROLLER"]["closed_loop"] = false;

        //recover urdf path
        std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
        auto robot = std::make_shared<robot_dart::Robot>(urdfs[0], packages);
        c_config["CONTROLLER"]["urdf"] = robot->model_filename();

        // get the controller
        auto controller_name = IWBC_CHECK(c_config["CONTROLLER"]["name"].as<std::string>());
        auto controller = inria_wbc::controllers::Factory::instance().create(controller_name, c_config);
        BOOST_CHECK(controller);

        auto p_controller = std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller);
        BOOST_CHECK(p_controller);
        BOOST_CHECK(!p_controller->tasks().empty());

        // get the behavior (trajectories)
        y::Node b_config = IWBC_CHECK(y::LoadFile(behaviors_paths[0]));
        auto behavior_name = IWBC_CHECK(b_config["BEHAVIOR"]["name"].as<std::string>());
        auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, controller, b_config);
        BOOST_CHECK(behavior);

        results.push_back(test_behavior(behavior, duration));
    }

    for (int i = 0; i < nexp; i++) {
        for (int j = 0; j < nexp; j++) {
            if (i != j) {
                compare_cmds(results[i], results[j]);
            }
        }
    }
}
