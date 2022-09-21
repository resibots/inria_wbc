#include <map>
#include <vector>

#include <boost/filesystem.hpp>

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

#include "utest.hpp"

namespace y = YAML;

std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> inline test_behavior(
    const std::shared_ptr<inria_wbc::behaviors::Behavior>& behavior, int duration)
{
    auto controller = behavior->controller();

    auto all_dofs = controller->all_dofs();
    auto controllable_dofs = controller->controllable_dofs();
    uint ncontrollable = controllable_dofs.size();

    std::vector<Eigen::VectorXd> cmds, cmds_filtered;
    inria_wbc::controllers::SensorData sensors;
    for (int i = 0; i < duration; ++i) {
        behavior->update(sensors); // default values for sensors
        auto cmd = controller->q(false);
        cmds.push_back(cmd);
        cmds_filtered.push_back(controller->filter_cmd(cmd).tail(ncontrollable));
    }
    return std::make_pair(cmds, cmds_filtered);
}

template <typename T>
inline void compare_cmds(utest::test_t test, const T& cmds, const T& cmds2)
{
    UTEST_INFO(test, "Comparing commands:");
    UTEST_CHECK(test, cmds.first.size() == cmds2.first.size());
    bool error = false;
    for (int i = 0; i < cmds.first.size(); ++i) {
        if ((cmds.first[i] - cmds2.first[i]).norm() >= 1e-8 || (cmds.second[i] - cmds2.second[i]).norm() >= 1e-8)
            error = true;
    }
    if (error)
        UTEST_ERROR(test, "Error, cmds are not the same")
    UTEST_INFO(test, "Comparison done");
}

void subtest(utest::test_t test, int nexp, const std::string& controller_path, const std::string& behavior_p, const std::string& urdf)
{
    UTEST_INFO(test, "Testing " + behavior_p + " determinism with " + urdf);
    std::vector<std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>>> results;
    int duration = 50000;

    for (int i = 0; i < nexp; i++) {
        y::Node c_config = IWBC_CHECK(y::LoadFile(controller_path));
        c_config["CONTROLLER"]["stabilizer"]["activated"] = false;
        c_config["CONTROLLER"]["compliance_posture"]["activated"] = false;
        c_config["CONTROLLER"]["collision_detection"]["activated"] = false;
        c_config["CONTROLLER"]["closed_loop"] = false;
        c_config["CONTROLLER"]["base_path"] = "../../etc/talos/";

        bool load_rand_yaml = rand() % 2 == 1;
        if (load_rand_yaml) {
            UTEST_INFO(test, "random yaml order");
            std::string tasks_path = "../../etc/talos/" + IWBC_CHECK(c_config["CONTROLLER"]["tasks"].as<std::string>());
            y::Node randomize = IWBC_CHECK(y::LoadFile(tasks_path));
            std::ofstream fout("../../etc/talos/randomize.yaml");
            fout << randomize;
            c_config["CONTROLLER"]["tasks"] = "randomize.yaml";
            fout.close();
        }

        //recover urdf path and mimic dof names
        std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
        auto robot = std::make_shared<robot_dart::Robot>(urdf, packages);
        c_config["CONTROLLER"]["urdf"] = robot->model_filename();
        c_config["CONTROLLER"]["mimic_dof_names"] = robot->mimic_dof_names();

        // get the controller
        auto controller_name = IWBC_CHECK(c_config["CONTROLLER"]["name"].as<std::string>());
        auto controller = inria_wbc::controllers::Factory::instance().create(controller_name, c_config);
        UTEST_CHECK(test, controller.get() != 0);

        auto p_controller = std::dynamic_pointer_cast<inria_wbc::controllers::PosTracker>(controller);
        UTEST_CHECK(test, p_controller.get() != 0);
        UTEST_CHECK(test, !p_controller->tasks().empty());

        // get the behavior (trajectories)
        y::Node b_config = IWBC_CHECK(y::LoadFile(behavior_p));
        auto behavior_name = IWBC_CHECK(b_config["BEHAVIOR"]["name"].as<std::string>());
        auto behavior = inria_wbc::behaviors::Factory::instance().create(behavior_name, controller, b_config);
        UTEST_CHECK(test, behavior.get() != 0);

        UTEST_INFO(test, "running behavior start");
        results.push_back(test_behavior(behavior, duration));
        UTEST_INFO(test, "running behavior end");
    }

    for (int i = 0; i < nexp; i++) {
        for (int j = 0; j < nexp; j++) {
            if (i != j) {
                compare_cmds(test, results[i], results[j]);
            }
        }
    }
}

int main(int argc, char** argv)
{
    utest::TestSuite test_suite;
    srand(time(NULL));

    std::string controller_path = "../../etc/talos/talos_pos_tracker.yaml";
    std::vector<std::string> behaviors_paths = {"../../etc/talos/arm.yaml", "../../etc/talos/squat.yaml"};
    std::vector<std::string> urdfs = {"talos/talos.urdf", "talos/talos_fast.urdf"};

    int nexp = 5;
    for (int b = 0; b < behaviors_paths.size(); b++) {
        for (int u = 0; u < urdfs.size(); u++) {
            auto test = utest::make_test("determinism_" + behaviors_paths[b] + "_" + urdfs[u]);
            UTEST_REGISTER(test_suite, test, subtest(test, nexp, controller_path, behaviors_paths[b], urdfs[u]));
        }
    }

    test_suite.run();
    utest::write_report(test_suite, std::cout, true);
    std::remove("../../etc/talos/randomize.yaml");
}